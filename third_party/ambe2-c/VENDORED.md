# ambe2-c (vendored)

AMBE+ / AMBE+2 speech decoder (blip25 decoder), used by the AMBE-3000R model of the DSP board (`c55/djxboard.cpp`).

* Source: https://github.com/tallcat4/ambe2-c, commit 70e32cb ("Update README.md"); only `decoder/include`,
  `decoder/src` and the license files are kept here.
* Local change: `decoder/src/ambeplus.c` `layer2()` / `layer2x()` read cos(pi k (l + 0.5) / L) from a table filled
  once instead of calling cos() about 4000 times per frame; the synthesized PCM is bit-identical.
* License: MIT (`LICENSE`, `decoder/LICENSE`).
