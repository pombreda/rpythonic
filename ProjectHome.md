RPythonic is a frontend for using [RPython](http://codespeak.net/pypy/dist/pypy/doc/coding-guide.html#restricted-python) (the translation toolchain of PyPy), it simplifies: wrapper generation, compiling standalone programs and Python extension modules.  It also generates wrappers for C libraries using ctypes that can be loaded by CPython2, CPython3, and PyPy.  Rpythonic also aims to advance the Rpython language by removing some of its limitations, and making it more useful as a general purpose language.

## Main Features ##
  * compiling RPython standalone executables
  * compiling RPython extension modules (for Python)
  * automatic integration with C libraries using ctypes generated wrappers