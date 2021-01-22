# oddio-engine
Engine Signal for the oddio audio library


test it here [oddio-engine_test](https://github.com/DasEtwas/oddio-engine_test)

## How it works

The Engine Signal is constructed using a list of RPMs and their corresponding engine sound loops. Optionally, crossfading can be set (this reduces a loop's playback time by the given crossfading value in seconds). When the RPM are set, the two loops of lower and higher RPM are each sampled with adjusted speeds and mixed depending on the RPM using a constant power mix.
