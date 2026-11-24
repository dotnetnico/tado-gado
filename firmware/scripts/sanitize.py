"""Run the native tests under the address and undefined-behaviour sanitizers."""

Import("env")  # noqa: F821

FLAGS = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
env.Append(CCFLAGS=FLAGS, LINKFLAGS=FLAGS)  # noqa: F821
