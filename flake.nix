{
  description = "Reproducible Godot GDExtension adapter for the standalone FABRIK core";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";

    fabrik-core = {
      url = "github:cabra-lat/fabrik-fortran/d63bed9067952dc758b00a0cfabe955dff9f2d41";
      flake = true;
    };

    godot-cpp = {
      url = "github:godotengine/godot-cpp/714c9e2c165db2dcb7e6ea57e62a04204d3cfbfa";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, fabrik-core, godot-cpp }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      packageFor = system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          core = fabrik-core.packages.${system}.default;
        in
        pkgs.stdenv.mkDerivation {
          pname = "fabrik-godot-adapter";
          version = "0.1.0";
          src = ./.;
          strictDeps = true;
          nativeBuildInputs = [ pkgs.gcc pkgs.cmake pkgs.ninja pkgs.gfortran pkgs.python3 ];
          dontConfigure = true;
          # Keep the C++ driver separate from the Fortran wrapper. Mixing the
          # gfortran package's g++ wrapper with the C ABI smoke test can change
          # the C++ ABI selected by CMake.
          CXX = "${pkgs.gcc}/bin/g++";
          CC = "${pkgs.gcc}/bin/gcc";
          buildPhase = ''
            runHook preBuild
            cmake -S . -B build -G Ninja \
              -DFABRIK_CORE_ROOT="${core}" \
              -DGODOT_CPP_SOURCE_DIR="${godot-cpp}" \
              -DCMAKE_BUILD_TYPE=Release
            cmake --build build --parallel "$NIX_BUILD_CORES"
            runHook postBuild
          '';
          doCheck = true;
          checkPhase = ''
            runHook preCheck
            ctest --test-dir build --output-on-failure
            runHook postCheck
          '';
          installPhase = ''
            runHook preInstall
            mkdir -p "$out/lib" "$out/bin" "$out/share/fabrik-godot-adapter"
            cp build/bin/libfabrik_adapter.* "$out/lib/"
            cp build/fabrik_cxx_smoke "$out/bin/"
            cp bin/fabrik_adapter.gdextension README.md docs/API.md "$out/share/fabrik-godot-adapter/"
            runHook postInstall
          '';
          passthru = {
            inherit core godot-cpp;
            cAbi = "flat C ABI smoke test is run in checkPhase";
          };
        };
    in {
      packages = nixpkgs.lib.genAttrs systems (system: {
        default = packageFor system;
      });
      checks = nixpkgs.lib.genAttrs systems (system: {
        default = self.packages.${system}.default;
      });
      devShells = nixpkgs.lib.genAttrs systems (system:
        let pkgs = nixpkgs.legacyPackages.${system}; in {
          default = pkgs.mkShell {
            packages = [ pkgs.cmake pkgs.ninja pkgs.gfortran pkgs.fortran-fpm pkgs.python3 ];
          };
        });
    };
}
