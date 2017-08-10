Import('env')

# Custom LST from ELF
env.Replace(
        OBJDUMP="avr-objdump",
)
env.AddPostAction(
    "$BUILD_DIR/firmware.elf",
    env.VerboseAction(" ".join([
        "$OBJDUMP",
        "-h",
        "-S",
        "$BUILD_DIR/firmware.elf",">" "$BUILD_DIR/firmware.lst"
    ]), "Dumping $BUILD_DIR/firmware.lst"),
)
