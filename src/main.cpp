#include <yaml.h>
#include <cassert>
#include <iostream>
#include <unistd.h>
#include <limits.h>

/*
./main --config config.yaml
*/
int main(int argc, char* argv[]) {

    // make args into a map
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; i += 2) {
        assert(argv[i][0] == '-' && argv[i][1] == '-');
        argv[i] += 2;
        args[argv[i]] = argv[i + 1];
    }

    // load config
    assert(args.find("config") != args.end());
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == NULL) {
        perror("getcwd() error");
        return 1;
    }
    YAML::Node config = yaml_solver(args["config"], std::string(cwd));

    return 0;
}