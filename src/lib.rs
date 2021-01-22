use oddio::{Controlled, Frame, Frames, Sample, Seek, Signal};
use std::cell::Cell;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

pub struct Engine<T> {
    loops: Vec<(f32, usize, Cell<f32>, Arc<Frames<T>>)>,
    shared: AtomicU32,
    prev_rpm: Cell<f32>,
    next_rpm: Cell<f32>,
    time_since_changed: Cell<f32>,
    rpm_smoothing_interval: f32,
    minimum_rpm: f32,
    maximum_rpm: f32,
}

impl<T> Engine<T>
where
    T: Frame + Copy,
{
    /// Apply dynamic gain to `signal`
    pub fn new(
        initial_rpm: f32,
        rpm_smoothing_interval: f32,
        loops: impl Iterator<Item = (f32, f32, Arc<Frames<T>>)>,
    ) -> Self {
        let loops = loops
            .map(|(loop_rpm, crossfade_duration, samples)| {
                let crossfade_duration =
                    (crossfade_duration * samples.rate() as f32).trunc() as usize;

                assert!(
                    crossfade_duration < samples.len() - 1,
                    "Crossfade exceeds maximum size"
                );

                (loop_rpm, crossfade_duration, Cell::new(0.0), samples)
            })
            .collect::<Vec<_>>();

        assert!(!loops.is_empty(), "At least one loop is required");

        let (minimum_rpm, maximum_rpm) = loops.iter().fold(
            (f32::INFINITY, f32::NEG_INFINITY),
            |(min, max), (rpm, _, _, _)| (min.min(*rpm), max.max(*rpm)),
        );

        Self {
            loops,
            shared: AtomicU32::new(initial_rpm.to_bits()),
            prev_rpm: Cell::new(initial_rpm),
            next_rpm: Cell::new(initial_rpm),
            time_since_changed: Cell::new(1e10),
            rpm_smoothing_interval,
            minimum_rpm,
            maximum_rpm,
        }
    }

    /// Returns linearly interpolated rpms
    #[inline]
    fn rpm(&self) -> f32 {
        let diff = self.next_rpm.get() - self.prev_rpm.get();
        let progress = (self.time_since_changed.get() / self.rpm_smoothing_interval).min(1.0);
        self.prev_rpm.get() + progress * diff
    }

    #[inline]
    fn advance_loops(&self, rpm: f32, interval: f32) {
        let a = interval * rpm;

        for (loop_rpm, crossfade_duration, position, samples) in self.loops.iter() {
            let samples_len = samples.len();
            let sampling_position = (position.get() + samples.rate() as f32 / loop_rpm * a)
                .rem_euclid((samples_len - crossfade_duration) as f32);
            position.set(sampling_position);
        }
    }

    /// Advances the `index`-th loop's position and samples it
    #[inline]
    fn sample(&self, index: usize) -> T {
        let (_, crossfade_duration, position, samples) = &self.loops[index];
        let sampling_position = position.get();

        let len = samples.len();

        let index = sampling_position.trunc() as usize;
        let fract = sampling_position.fract() as f32;

        if index >= len / 2 {
            let i = index + crossfade_duration + len / 2;
            //println!("C {:6}", i % len);

            lerp(&samples[i % len], &samples[(i + 1) % len], fract)
        } else if index > len / 2 - crossfade_duration {
            let crossfade =
                (index - len / 2 + crossfade_duration) as f32 / *crossfade_duration as f32;

            let i0 = index + len / 2;
            let i1 = index + crossfade_duration + len / 2;

            //println!("B {:6} {:6}, {:5.3}", i0 % len, i1 % len, crossfade);

            lerp(
                &lerp(&samples[i0 % len], &samples[(i0 + 1) % len], fract),
                &lerp(&samples[i1 % len], &samples[(i1 + 1) % len], fract),
                crossfade,
            )
        } else {
            let i = index + len / 2;
            //println!("A {:6}", i % len);

            lerp(&samples[i % len], &samples[(i + 1) % len], fract)
        }
    }
}

impl<T: Frame + Copy> Signal for Engine<T> {
    type Frame = T;

    #[allow(clippy::float_cmp)]
    fn sample(&self, interval: f32, out: &mut [T]) {
        let shared = f32::from_bits(self.shared.load(Ordering::Relaxed));
        if self.next_rpm.get() != shared {
            self.prev_rpm.set(self.rpm());
            self.next_rpm.set(shared);
            self.time_since_changed.set(0.0);
        }

        let mut lower_rpm = self.minimum_rpm;
        let mut higher_rpm = self.minimum_rpm;
        let mut lower_index = 0;
        let mut higher_index = 0;

        let mut rpm = self.rpm();
        for x in out {
            *x = if rpm <= self.minimum_rpm {
                self.sample(0)
            } else if rpm >= self.maximum_rpm {
                self.sample(self.loops.len() - 1)
            } else {
                if rpm < lower_rpm {
                    let mut last_rpm = lower_rpm;
                    while lower_index > 0 {
                        lower_index -= 1;
                        let loop_rpm = self.loops[lower_index].0;

                        if loop_rpm < rpm {
                            higher_rpm = last_rpm;
                            higher_index = lower_index + 1;
                            lower_rpm = loop_rpm;

                            break;
                        }
                        last_rpm = loop_rpm;
                    }
                }
                if rpm > higher_rpm {
                    let mut last_rpm = higher_rpm;
                    while higher_index < self.loops.len() {
                        higher_index += 1;
                        let loop_rpm = self.loops[higher_index].0;

                        if loop_rpm > rpm {
                            lower_rpm = last_rpm;
                            lower_index = higher_index - 1;
                            higher_rpm = loop_rpm;

                            break;
                        }
                        last_rpm = loop_rpm;
                    }
                }

                let mix_ratio = (rpm - lower_rpm) / (higher_rpm - lower_rpm);
                let lower_gain = (1.0 - mix_ratio).sqrt();
                let higher_gain = mix_ratio.sqrt();

                mix(
                    &scale(&self.sample(lower_index), lower_gain),
                    &scale(&self.sample(higher_index), higher_gain),
                )
            };

            rpm = self.rpm();
            self.time_since_changed
                .set(self.time_since_changed.get() + interval);

            self.advance_loops(rpm, interval);
        }
    }

    fn remaining(&self) -> f32 {
        f32::INFINITY
    }
}

impl<T: Frame + Copy> Seek for Engine<T> {
    fn seek_to(&self, t: f32) {
        self.advance_loops(self.rpm(), -t);
    }
}

pub struct EngineControl<'a, T>(&'a Engine<T>);

unsafe impl<'a, T: 'a> Controlled<'a> for Engine<T> {
    type Control = EngineControl<'a, T>;

    fn make_control(signal: &'a Engine<T>) -> Self::Control {
        EngineControl(signal)
    }
}

impl<'a, T> EngineControl<'a, T> {
    /// Get the current RPM
    pub fn rpm(&self) -> f32 {
        f32::from_bits(self.0.shared.load(Ordering::Relaxed))
    }

    /// Adjust the RPM
    pub fn set_rpm(&mut self, factor: f32) {
        self.0.shared.store(factor.to_bits(), Ordering::Relaxed);
    }
}

fn map<T: Frame>(x: &T, mut f: impl FnMut(Sample) -> Sample) -> T {
    let mut out = T::ZERO;
    for (&x, o) in x.channels().iter().zip(out.channels_mut()) {
        *o = f(x);
    }
    out
}

fn bimap<T: Frame>(x: &T, y: &T, mut f: impl FnMut(Sample, Sample) -> Sample) -> T {
    let mut out = T::ZERO;
    for ((&x, &y), o) in x
        .channels()
        .iter()
        .zip(y.channels())
        .zip(out.channels_mut())
    {
        *o = f(x, y);
    }
    out
}

pub(crate) fn lerp<T: Frame>(a: &T, b: &T, t: f32) -> T {
    bimap(a, b, |a, b| a + t * (b - a))
}

pub(crate) fn mix<T: Frame>(a: &T, b: &T) -> T {
    bimap(a, b, |a, b| a + b)
}

pub(crate) fn scale<T: Frame>(x: &T, factor: f32) -> T {
    map(x, |x| x * factor)
}
