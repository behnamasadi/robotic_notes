#ifndef ARGUMENTS_PARSER_HPP
#define ARGUMENTS_PARSER_HPP

#include <algorithm>
#include <string>
#include <vector>

///
/// A simple utility command line parser for parsing parameters send to the
/// application
///

class ArgumentsParser {
private:
  std::vector<std::string> m_arguments;

public:
  ArgumentsParser(int &argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
      m_arguments.emplace_back(std::string(argv[i]));
    }
  }

  const std::string getArg(const std::string &arg) const {
    auto itr = std::find(m_arguments.begin(), m_arguments.end(), arg);
    if (itr != m_arguments.end() && ++itr != m_arguments.end()) {
      return *itr;
    }
    return std::string("");
  }
  bool argExists(const std::string &arg) const {
    return std::find(m_arguments.begin(), m_arguments.end(), arg) !=
           m_arguments.end();
  }
};

#endif
