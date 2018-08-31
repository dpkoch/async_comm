#!/usr/bin/env python3

import os
import subprocess
import sys

import urllib.request
import yaml

from termcolor import cprint
print_blue    = lambda x: cprint(x, 'blue',    attrs=['bold'])
print_cyan    = lambda x: cprint(x, 'cyan',    attrs=['bold'])
print_green   = lambda x: cprint(x, 'green',   attrs=['bold'])
print_magenta = lambda x: cprint(x, 'magenta', attrs=['bold'])
print_red     = lambda x: cprint(x, 'red',     attrs=['bold'])
print_yellow  = lambda x: cprint(x, 'yellow',  attrs=['bold'])

ROS_DISTROS = ['kinetic', 'lunar', 'melodic']

REPO_URL = 'https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config'
BRANCH = 'production'
INDEX_FILE = 'index.yaml'

def run_test(target, branch):
    test_name = "%(ros_distro)s_%(os_distro)s_%(os_release)s_%(arch)s" % target

    config = target
    config['branch'] = branch
    config['test_name'] = test_name

    # move working directory into test folder
    os.mkdir(test_name)
    os.chdir(test_name)

    # generate prerelease scripts
    print("Generating prerelease scripts...")
    generate_prerelease_command = """
    generate_prerelease_script.py \\
      https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \\
      %(ros_distro)s default %(os_distro)s %(os_release)s %(arch)s \\
      --custom-repo \\
      async_com__custom-2:git:https://github.com/dpkoch/async_comm.git:%(branch)s \\
      --level 0 \\
      --output-dir ./
    """ % config

    with open("../%s-generate.log" % (test_name), 'w') as log_file:
        process = subprocess.run(generate_prerelease_command, shell=True, stdout=log_file, stderr=subprocess.STDOUT)
        if process.returncode != 0:
            print_yellow("Generating prerelease scripts failed!")
            os.chdir('..')
            print_red("[Failed]")
            return False

    # run prerelease test
    print("Running prerelease tests...")
    with open("../%s-test.log" % (test_name), 'w') as log_file:
        process = subprocess.run("./prerelease.sh", stdout=log_file, stderr=subprocess.STDOUT)
        os.chdir('..')
        if process.returncode == 0:
            print_green("[Passed]")
            return True
        else:
            print_red("[Failed]")
            return False

def get_repo_file(repo_path):
    url = '/'.join([REPO_URL, BRANCH, repo_path])
    with urllib.request.urlopen(url) as response:
        return response.read()

def get_prerelease_targets():
    targets = []
    index_yaml = yaml.load(get_repo_file(INDEX_FILE))
    for ros_distro in ROS_DISTROS:
        config_files = index_yaml['distributions'][ros_distro]['release_builds']
        for file_path in config_files.values():
            config_yaml = yaml.load(get_repo_file(file_path))
            for os_distro in config_yaml['targets']:
                for os_release in config_yaml['targets'][os_distro]:
                    for arch in config_yaml['targets'][os_distro][os_release]:
                        targets.append({'ros_distro': ros_distro, \
                                        'os_distro': os_distro, \
                                        'os_release': os_release, \
                                        'arch': arch})
    return targets

def run_prerelease_tests(branch):

    print_magenta("Running all tests for the \"%s\" branch" % (branch))

    print()
    print_magenta("Running tests for the following ROS distros:")
    for distro in ROS_DISTROS:
        print(distro)

    print()
    print("Retrieving release targets...")
    targets = get_prerelease_targets()

    print()
    print_magenta("The following release targets will be tested:")
    for target in targets:
        print("%(ros_distro)s %(os_distro)s %(os_release)s %(arch)s" % target)

    # run all tests and aggregate overall result into exit code
    failed_tests = 0
    for target in targets:
        print()
        print_blue("Testing %(ros_distro)s %(os_distro)s %(os_release)s %(arch)s" % target)
        if not run_test(target, branch):
            failed_tests += 1

    # print and return overall result
    print()
    if failed_tests > 0:
        print_red("Failed %d of %d tests." % (failed_tests, len(targets)))
        return False
    else:
        print_green("Passed %d tests." % (len(targets)))
        return True

if __name__ == '__main__':

    # check that branch argument has been passed in
    if len(sys.argv) != 2:
        print("Usage: %s <branch>" % (sys.argv[0]))
        exit(2)
    branch = sys.argv[1]

    # check that working directory is empty
    if os.listdir('.'):
        print("Non-empty directory, please run in an empty directory. Aborting.")
        exit(3)

    exit_code = 0 if run_prerelease_tests(branch) else 1
    print_cyan("Exit code: %d" % (exit_code))
    exit(exit_code)
