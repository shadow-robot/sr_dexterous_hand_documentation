# Contributing Guide

## Coding guidelines
- **Do NOT push your code to the main branch**: code that has not been reviewed by someone else can not be pushed to the main branch. When you start a feature, you create a new branch and you keep working in that branch until you're satisfied the feature is ready, documented, etc... then you send a pull request and wait for someone else to review the code and merge it.
If you work for Shadow Robot, you should name the branch: 
  - When adding a feature: `F#SRC-123_descriptive_name`
  - When solving a bug: `B#SRC-123_descriptive_name` 

  where 123 is the number of the JIRA task. It'll automatically update JIRA for you. 
- **Send skeleton pull requests**: when developing a new feature, especially if it implies creating a new ros package, please create a first skeleton of your code (no algorithm in there, just the package, a few bare classes, the connections). Make sure you have a proper `README.md` that explains how to run the code, the connections and any other important facts.
- **Keep the pull requests as focused as possible**: merging a long pull request is very painful. Keep them as short as possible. It's much better to have multiple smaller pull requests to merge than a big one.
- **Cosmetics pull requests should be separate**: if for some reason you need to do some cosmetic change (whitespace, indentation), please do so in a separate code request. 

## Conventions
We stick to the ROS conventions for python and c++. As mentioned by the ROS documentation: 
>Coding style is important. A clean, consistent style leads to code that is more readable, debuggable, and maintainable. We strive to write elegant code that will not only perform its desired function today, but will also live on, to be re-used and improved by other developers for many years to come.

Some of them, might be detected by our code checks but some are the developer's (and reviewer's) responsability to follow (e.g. private variables and functions convention in python)

### Python
Documentation found here: http://wiki.ros.org/PyStyleGuide

#### Code style
A quick summary of the code style here:

- `package_name`
- `topic_name`
- `service_name`
- `file_name`
- `ClassName`
- `method_name`
- `field_name`
- `_private_something`
- `self.__really_private_field`
- `_global`
- 4 space indentation
- don't leave trailing whitespaces at the end of your lines

#### Package/Module Names
http://wiki.ros.org/PyStyleGuide#Package.2BAC8-Module_Names_.28__init__.py_files.29

#### Node Files
http://wiki.ros.org/PyStyleGuide#Node_Files

### C++
Documentation found here: http://wiki.ros.org/CppStyleGuide

A quick summary of the code style here:
- `package_name`
- `topic_name`
- `service_name`
- `liblibrary_name` (Note that you shouldn't insert underscore immediately after the lib prefix in the library name)
- `file_name`
- `ClassName`
- `method_name(int argument_name)`
- `normal_variable`
- `member_variable_`
- `g_global_variable_name` (Global variables should almost never be used)
- `CONSTANT_NAME`
- `namespace_name`
- 2 space indentation
- don't leave trailing whitespaces at the end of your lines
- go to the next line before a bracket (c++ obviously)

### ROS package naming
- The names must follow the standard ROS naming conventions for packages (lower case, start with a letter, use underscore separators). See http://wiki.ros.org/ROS/Patterns/Conventions#Packages
- Package names need to be chosen carefully to minimise the chance of name collision with existing packages. A suggestion to make the names unique is to use a prefix (sr_ in the case of shadow robot packages, e.g. sr_msgs).
- If you work for Shadow Robot, you should follow the specified Repository Naming Conventions.

## Licenses
If you work for Shadow Robot, you should follow the specified Licenses conventions.
