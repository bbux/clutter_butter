# C++ Boilerplate
[![Build Status](https://travis-ci.org/bbux/clutter_butter.svg?branch=master)](https://travis-ci.org/bbux/clutter_butter)
[![Coverage Status](https://coveralls.io/repos/github/bbux/clutter_butter/badge.svg?branch=master)](https://coveralls.io/github/bbux/clutter_butter?branch=master)
---

## Overview

Final Project - Toy Jail Robot Demo


## Backlog

Created using SIP ([Solo Iterative Processess](http://www.cs.wayne.edu/rajlich/SlidesSE/18%20example%20of%20sip.pdf))

[Backlog](https://docs.google.com/spreadsheets/d/1wChuRU8l6yA1EAUHQB64F89dBjVsw7t1enh5LzcWQo4/edit#gid=1120123239)

## License

Licensed under the [MIT License](https://opensource.org/licenses/MIT)
 
## Building



## Tests

### Test Documentation

### Unit Tests

To run the unit tests, first build the application as described in the build section.  There will be a folder created called test.  Run the following command to run the unit tests:

```
test/cpp-test
```

The results should be a text visualization describing the test results.

### Code Coverage

In order to generate code coverage lcov needs to be installed:

```
sudo apt-get install lcov
```

To generate the test coverage:

```
cd build/
cmake -DCOVERAGE=ON -DCMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```

The results can be viewed by opening coverage/index.html in a web browser

## Doxygen Documentation

To generate the doxygen documentation from the root directory:

```
doxygen doxygen.config
```

This will create the documentation in the docs directory.

## TODO


## References
