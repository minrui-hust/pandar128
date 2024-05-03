#include "argparse.hpp"
#include "driver.h"

argparse::ArgumentParser &parse_args(int argc, char **argv) {
  static argparse::ArgumentParser parser("pandar128");
  parser.add_argument("cfg").action(
      [](const std::string &path) { return YAML::LoadFile(path); });

  try {
    parser.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << parser;
    std::exit(1);
  }
  return parser;
}

int main(int argc, char **argv) {
  auto &args = parse_args(argc, argv);

  auto cfg = args.get<YAML::Node>("cfg");

  pandar128::Driver driver(cfg);

  driver.start();

  return 0;
}
