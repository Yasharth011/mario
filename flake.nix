{
  description = "flake for an autonomous rover";

  inputs = {

    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";

    flake-utils.url = "github:numtide/flake-utils";

  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let pkgs = nixpkgs.legacyPackages.${system};
      in rec {

        packages.yasmin = with pkgs;
          stdenv.mkDerivation {
            name = "yasmin";
            src = fetchFromGitHub {
              owner = "Yasharth011";
              repo = "yasmin_UNROS";
              rev = "95bcb301809f84e47df8db38c385c4a4318e0ccb"; # v3.3.0
              sha256 = "sha256-8yKFlNrmOJpScHkj7gHhL8UlP7JZ8aJFUXTrMq4I7gw=";
            };
            nativeBuildInputs = [ cmake ];
            configurePhase = ''
              	    cd yasmin 
                          mkdir build && cd build 
                          cmake .. -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out
            '';
            buildPhase = "  make ";
            installPhase = ''
              make install
            '';
          };
packages.opencv = pkgs.opencv.override {
          enableGtk2 = true;
        };
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            cmake
            packages.yasmin
            librealsense
            onnxruntime
            packages.opencv
	    boost
	    taskflow
          ];
        };

      });
}
