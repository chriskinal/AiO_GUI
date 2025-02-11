Import("env")
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "binary", "-R", ".eeprom",
        "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.bin"
    ]), "Building $BUILD_DIR/${PROGNAME}.bin")
)
env.Execute("cp %s %s" % ("$BUILD_DIR/${PROGNAME}.hex", "$PROJECT_DIR/compiled_firmware/"))
env.Execute("cp %s %s" % ("$BUILD_DIR/${PROGNAME}.bin", "$PROJECT_DIR/compiled_firmware/"))