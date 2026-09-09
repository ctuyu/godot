"""Functions used to generate source files during build time."""

import methods


def cap_files_builder(target, source, env):
    # The names are baked in at build time because a test binary has no anchored
    # repo root and cannot glob the source tree at run time.
    names = sorted(source[0].read())
    with methods.generated_wrapper(str(target[0])) as file:
        file.write("inline constexpr const char *CAP_SOURCE_FILES[] = {\n")
        for name in names:
            file.write(f'\t"{name.rsplit("/", 1)[-1]}",\n')
        file.write("};\n")
