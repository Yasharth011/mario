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
                "https://github.com/rerun-io/rerun/releases/download/0.28.1/rerun_cpp_sdk.zip";
              hash = "sha256-hTewsXKIsHte7ER/vf1s0QVK62rDNufm16jZTE0DdP8=";
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

        packages.stella_vslam = with pkgs;
          stdenv.mkDerivation {
            name = "stella_vslam";
            src = fetchFromGitHub {
              owner = "stella-cv";
              repo = "stella_vslam";
              rev = "7b78cc95a9961bcbd6408749ea750e37cec45541";
              sha256 = "sha256-kV29ALRrNQcrJ+OOeXFgHUcMIcbI9Lqxc0RzkEd/qas=";
              fetchSubmodules = true;
            };
            cmakeFlags = [ "-DUSE_AVX=OFF" ];
            nativeBuildInputs = [ cmake pkg-config ];
            buildInputs = [ eigen pangolin ];
            propagatedBuildInputs =
              [ yaml-cpp sqlite.dev g2o.dev packages.opencv glew ];
          };

        packages.libzmq = with pkgs;
          stdenv.mkDerivation {
            name = "libzmq";
            src = fetchFromGitHub {
              owner = "zeromq";
              repo = "libzmq";
              rev = "622fc6dde99ee172ebaa9c8628d85a7a1995a21d"; # v4.3.5
              sha256 = "sha256-q2h5y0Asad+fGB9haO4Vg7a1ffO2JSb7czzlhmT3VmI=";
            };
            nativeBuildInputs = [ cmake catch2 ];
            configurePhase = ''
              mkdir build && cd build 
              cmake .. -DCMAKE_INSTALL_PREFIX=$out
            '';
            installPhase = ''
              make -j4 install
            '';
          };

        packages.cppzmq = with pkgs;
          stdenv.mkDerivation {
            name = "cppzmq";
            src = fetchFromGitHub {
              owner = "zeromq";
              repo = "cppzmq";
              rev = "3bcbd9dad2f57180aacd4b4aea292a74f0de7ef4"; # v4.11.0
              sha256 = "sha256-c6IZ5PnuB96NLYHDHdNclYSF4LpqAfFWxVzeP8BzhCE=";
            };
            nativeBuildInputs = [ cmake ];
            buildInputs = [ packages.libzmq ];
            configurePhase = ''
              mkdir build && cd build 
              cmake .. -DCPPZMQ_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$out
            '';
            installPhase = ''
              make -j4 install
            '';
          };

        packages.ompl = with pkgs;
          stdenv.mkDerivation {
            name = "ompl";
            src = fetchFromGitHub {
              owner = "ompl";
              repo = "ompl";
              rev = "eabb4d0f825666e4553a2ec0ecb9c7a5a33637dd";
              sha256 = "sha256-mm6tVoCdFk/+rdOzeONI/rMdU2FUHVw1ESpp289u1B0=";
            };
            nativeBuildInputs = [ cmake ];
            propagatedBuildInputs = [ boost eigen yaml-cpp ];
            configurePhase = ''
              mkdir -p build/Release && cd build/Release 
              cmake ../.. -DCMAKE_INSTALL_PREFIX=$out
            '';
            installPhase = ''
              make -j4 install
            '';
          };

        packages.grid_map_core = with pkgs;
          stdenv.mkDerivation {
            name = "grid_map_core";
            src = fetchFromGitHub {
              owner = "Yasharth011";
              repo = "grid_map_UNROS";
              rev = "1cf4564a193abda60cc9ffbfd4a33ac48ca2c248";
              sha256 = "sha256-W7OVQPlM259awMx/ZW8S107HotXxqKrnF8jbq6IDSGk=";
            };
            nativeBuildInputs = [ cmake ];
            propagatedBuildInputs = [ eigen ];
            configurePhase = ''
	      cd grid_map_core
              mkdir -p build && cd build 
              cmake .. -DCMAKE_INSTALL_PREFIX=$out
            '';
          };

        packages.grid_map_pcl = with pkgs;
          stdenv.mkDerivation {
            name = "grid_map_pcl";
            src = fetchFromGitHub {
              owner = "Yasharth011";
              repo = "grid_map_UNROS";
              rev = "1cf4564a193abda60cc9ffbfd4a33ac48ca2c248";
              sha256 = "sha256-W7OVQPlM259awMx/ZW8S107HotXxqKrnF8jbq6IDSGk=";
            };
            nativeBuildInputs = [ cmake ];
            propagatedBuildInputs = [ boost eigen pcl yaml-cpp spdlog packages.grid_map_core ];
            configurePhase = ''
	      cd grid_map_pcl
              mkdir -p build && cd build 
              cmake .. -DCMAKE_INSTALL_PREFIX=$out
            '';
          };

        devShells.default = pkgs.mkShell {
          nativeBuildInputs = with pkgs; [ cmake pkg-config ];
          buildInputs = with pkgs; [
            packages.yasmin
            onnxruntime
            packages.rerun_cpp
            asio
            taskflow
            packages.cobs-c
            packages.stella_vslam
            packages.cppzmq
	    packages.libzmq
	    spdlog
	    packages.ompl
	    packages.grid_map_core
	    yaml-cpp
	    pcl
	    librealsense
          ];
        };

      });
}
