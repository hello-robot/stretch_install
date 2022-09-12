# Contributing to Stretch Install

Thank you for considering contributing to this repository. Stretch Install houses bash scripts and tutorials that enable users to setup/configure their robots. This guide explains the layout of this repo and how best to make and test changes.

## Repo Layout

 * `README.md` & `LICENSE.md` - includes info about the repo and a table of tutorials available
 * `stretch_new_*_install.sh` - high level scripts meant to be run by the user
 * `factory/` - subscripts and assets not meant to be run by the user
   * `18.04/` - subscripts and assets specific to performing a Ubuntu 18.04 software install
     * `stretch_initial_setup.sh` - a bunch of checks and initial setup that are run before performing a robot install
     * `stretch_install_*.sh` - helper scripts that install a specific set of packages
     * `stretch_create_*_workspace.sh` - creates a ROS/ROS2 workspace
     * `stretch_ros*.repos` - the ROS packages that are included and compiled in the ROS workspace by the `stretch_create_*_workspace.sh` script
     * `hello_robot_*.desktop` - autostarts programs to run when the robot boots up
   * `<>.04/` - Ubuntu <>.04 software install related subscripts/assets. Similar in layout to 18.04/
 * `docs/` - contains tutorials for using the scripts in this repo

Once you're ready to make changes to this repo, you can [fork it on Github](https://github.com/hello-robot/stretch_docs/fork).

## Contributing to the tutorials

The tutorials in the `docs/` folder are markdown files that get rendered into our https://docs.hello-robot.com site. If you edit them and file a pull request towards this repo, the changes to the tutorials will get reflected on the docs site. In order to live preview changes to these tutorials, first install mkdocs using:

```bash
python3 -m pip install mkdocs mkdocs-material mkdocstrings==0.17.0 pytkdocs[numpy-style] jinja2=3.0.3
```

Then, run the dev server using:

```
python3 -m mkdocs serve
```

Now you can make additions or changes to the source markdown files and see the changes reflected live in the browser.

## Contributing to the installation scripts

If you are looking to change scripts/assets of an existing software installation (e.g. Ubuntu 18.04/20.04), look within the `factory/<>.04/` directory and make changes to the appriopriate files. If you're looking to add support for a new Ubuntu distro (e.g. Ubuntu 19.04), create `factory/19.04` with assets from a previous installation and tweak the scripts until they works correctly for the Ubuntu distro you are targeting. Then, edit the high level scripts (e.g. `stretch_new_*_install.sh`) to call your distro's specific assets correctly. Ensuring that the tutorials in the `docs/` work for your new distro is a good way to ensure that your `factory/<>.04/` directory works correctly. Since bash scripts change behavior based on the underlying system, it can be helpful to use containers to create reproducible behaviors while you're developing support for the new distro. [Multipass](https://multipass.run/install) works well on Ubuntu systems. You can create a new container emulating any Ubuntu distro using the command:

```bash
multipass launch -c 6 -d 30G -m 7G -n <container-name> 19.04
```

Swap `19.04` in the above command with the distro you're targeting. The above command creates a containers with 30GB disk space, 7GB swap, 6 cores, and the name `<container-name>`. We've found that at least 30GB disk space is needed for the Ubuntu 18.04/20.04 installations.

Then, you can access the shell of your new container using:

```bash
multipass shell <container-name>
```

Other helpful multipass subcommand include `transfer`, which allows you to transfer files to the container, and `delete`, which allows you to delete the container. See the [multipass docs](https://multipass.run/docs) for more details.

## Filing a Pull Request

Once your changes are committed to your fork, you can [open a pull request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) towards the Stretch Install master branch. A member of Hello Robot's software team will review your new PR and get it merged and available for all Stretch users.
