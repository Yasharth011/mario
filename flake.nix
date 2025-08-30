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

        packages.opencv = pkgs.opencv.override { enableGtk2 = true; };

        packages.cobs-c = with pkgs;
          stdenv.mkDerivation {
            name = "cobs-c";
            src = fetchFromGitHub {
              owner = "cmcqueen";
              repo = "cobs-c";
              rev = "6cc55cddb06568bc026ed85f8e5f433496a3622c";
              sha256 = "sha256-aIWT5w3KUHEzxiWuHlfNWuxvjuCGX2nCBFYHNmYc2Is=";
            };
            nativeBuildInputs = [ pkg-config validatePkgConfig autoreconfHook ];
            passthru.tests.pkg-config = testers.hasPkgConfigModule {
              package = finalAttrs.finalPackage;
              moduleName = "cobs";
            };
          };

        packages.rerun_cpp = with pkgs;
          stdenv.mkDerivation {
            name = "rerun_cpp";
            src = fetchzip {
              url =
                "https://github.com/rerun-io/rerun/releases/download/0.24.1/rerun_cpp_sdk.zip";
              hash = "sha256-FzoLGeZMp5N5VRNT+QO8u7XobhxrqM1TyNVfAXsIHTY=";
            };
            nativeBuildInputs = [ cmake pkg-config ];
            propagatedBuildInputs = [ arrow-cpp ];
            configurePhase = ''
              mkdir build && cd build
              cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$out \
              -DRERUN_DOWNLOAD_AND_BUILD_ARROW=OFF -DRERUN_ARROW_LINK_SHARED=ON
            '';
            meta = { description = "C++ bindings for rerun.io"; };
          };

        packages.path-planning = with pkgs;
          stdenv.mkDerivation {
            name = "path-planning";
            src = fetchFromGitHub {
              owner = "CPPavithra";
              repo = "PathPlanning-Astar";
              rev = "becfd3604c8a8f327c1f7650f455072fad78da1a"; # v0.1
              sha256 = "sha256-5bRL6/ZkN/Lw6d8dZl30Odg1q9l4NT8ctA1zR531WOU=";
            };
            nativeBuildInputs = [ cmake ];
            propagatedBuildInputs = [
	      pkgs.eigen
              pkgs.librealsense
	      pkgs.boost
              pkgs.pcl
	      packages.rerun_cpp
              packages.opencv
            ];
            configurePhase = ''
                          mkdir build && cd build 
                          cmake .. -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out
            '';
            buildPhase = "  make ";
            installPhase = ''
              make install
            '';
          };

        devShells.default = pkgs.mkShell {
          nativeBuildInputs = with pkgs; [ cmake pkg-config ];
          buildInputs = with pkgs; [
            packages.yasmin
            # librealsense
            onnxruntime
            # packages.opencv
            # boost
            asio
            taskflow
            # eigen
            # pcl
            packages.cobs-c
            packages.path-planning
          ];
        };

      });
}
