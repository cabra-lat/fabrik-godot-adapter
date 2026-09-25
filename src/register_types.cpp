#include "fabrik_chain_3d.h"
#include "fabrik_rig_3d.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_fabrik_adapter(ModuleInitializationLevel level) {
    if (level == MODULE_INITIALIZATION_LEVEL_SCENE) {
        ClassDB::register_class<FabrikChain3D>();
        ClassDB::register_class<FabrikRig3D>();
    }
}

void uninitialize_fabrik_adapter(ModuleInitializationLevel level) {
    if (level == MODULE_INITIALIZATION_LEVEL_SCENE) {
        // No global state to tear down.
    }
}

extern "C" {
GDExtensionBool GDE_EXPORT fabrik_adapter_library_init(
    GDExtensionInterfaceGetProcAddress get_proc_address,
    GDExtensionClassLibraryPtr library,
    GDExtensionInitialization *initialization) {
    GDExtensionBinding::InitObject init_object(get_proc_address, library, initialization);
    init_object.register_initializer(initialize_fabrik_adapter);
    init_object.register_terminator(uninitialize_fabrik_adapter);
    init_object.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_object.init();
}
}
