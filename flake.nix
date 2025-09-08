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
            buildInputs = [
              eigen
              librealsense
              boost
              pcl
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
        packages.ixwebsocket = with pkgs;
          stdenv.mkDerivation {
            name = "ixwebsocket";
            src = fetchFromGitHub {
              owner = "machinezone";
              repo = "IXWebSocket";
              rev = "c5a02f1066fb0fde48f80f51178429a27f689a39";
              sha256 = "sha256-j/Fa45es2Oj09ikMZ8rMsSEOXLgO5+H7aqNurOua9LY=";
            };
            patches = [
              (fetchpatch {
                # Need to patch CMakeLists for using SpdLog from propagated inputs
                url =
                  "https://github.com/kknives/IXWebSocket/commit/73f5d8d4cec5a336f642a03d2d067cd8acee17dc.patch";
                hash = "sha256-RulJ2k6FF0X/d4ZVWBIf2gXmLMhVRQQeRjscDdhzNdk=";
              })
            ];
            nativeBuildInputs = [ cmake pkg-config spdlog ];
            propagatedBuildInputs = [ openssl.dev zlib.dev curl.dev ];
            configurePhase = ''
              mkdir build && cd build
              cmake .. -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out \
              -DBUILD_SHARED_LIBS=ON -DUSE_ZLIB=1 -DUSE_TLS=1 -DUSE_WS=1
            '';
            installPhase = ''
              make install
            '';
            meta = { description = "ixwebsocket"; };
          };
        packages.morb_slam = with pkgs;
          stdenv.mkDerivation {
            name = "morb_slam";
            src = fetchFromGitHub {
              owner = "kknives";
              repo = "MORB_SLAM";
              rev = "b6f883605aca6f65dafc6af38f02df8763b91385";
              sha256 = "sha256-o0h0dCrAt+/Q2f29cGGFn1Yt1wb8yUBA4pxvrwV9jAw=";
            };
            nativeBuildInputs = [ cmake pkg-config copyPkgconfigItems ];
            pkgconfigItems = [
              (makePkgconfigItem rec {
                name = "MORB_SLAM";
                version = "3.0.0";
                cflags = [
                  "-I${variables.cameradir} -I${variables.dbow2dir} -I${variables.sophusdir} -I${variables.g2odir} -I${variables.includedir}"
                ];
                libs = [
                  "-L${variables.lddir} -lMORB_SLAM -L${variables.dbow2lddir} -lDBoW2 -L${variables.g2olddir} -lg2o"
                ];
                variables = rec {
                  prefix = "${placeholder "out"}";
                  includedir = "${prefix}/include/source/include";
                  cameradir = "${prefix}/include/source/include/CameraModels";
                  dbow2dir = "${prefix}/include/source/Thirdparty/DBoW2";
                  sophusdir = "${prefix}/include/source/Thirdparty/Sophus";
                  g2odir = "${prefix}/include/source/Thirdparty/g2o";
                  lddir = "${prefix}/lib";
                  dbow2lddir =
                    "${prefix}/include/source/build/Thirdparty/DBoW2/lib";
                  g2olddir =
                    "${prefix}/include/source/build/Thirdparty/g2o/lib";
                };
              })
            ];
            passthru.tests.pkg-config = testers.hasPkgConfigModule {
              package = finalAttrs.finalPackage;
              moduleName = "MORB_SLAM";
            };
            buildInputs = [
              eigen
              pangolin
              packages.ixwebsocket
              glew
              gdal
              packages.opencv
              boost
            ];
            # configurePhase = ''
            # ./build.sh
            # '';
            configurePhase = ''
              mkdir build && cd build
              mkdir -p Thirdparty/DBoW2 Thirdparty/g2o Thirdparty/Sophus

              cd Thirdparty/DBoW2
              cmake ../../../Thirdparty/DBoW2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out
              make -j
              mkdir -p $out/lib/Thirdparty/DBoW2/lib
              # cp /build/source/Thirdparty/DBoW2/lib/libDBoW2.so $out/lib/Thirdparty/DBoW2/lib

              cd ../g2o
              cmake ../../../Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out
              make -j
              mkdir -p $out/lib/Thirdparty/g2o/lib
              # cp /build/source/Thirdparty/g2o/lib/libg2o.so $out/lib/Thirdparty/g2o/lib

              cd ../Sophus
              cmake ../../../Thirdparty/Sophus -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out -DBUILD_TESTS=OFF
              make -j

              cd ../..
              cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out
            '';
            buildPhase = ''
              make -j6
            '';
            installPhase = ''
              runHook preInstall
              make install
              rm -f $out/include/source/build/libMORB_SLAM.so
              mkdir -p $out/include/CameraModels
              mv $out/include/include/CameraModels/* $out/include/CameraModels
              rm -rf $out/include/include/CameraModels
              mv $out/include/include/* $out/include
              runHook postInstall
              # prev_rpath=$(patchelf --print-rpath libMORB_SLAM.so | sed 's#/build/source#'$out/lib#g)
              # echo "Modified rpath=$prev_rpath"
              # patchelf --set-rpath $prev_rpath libMORB_SLAM.so
            '';
            meta = { description = "morb_slam"; };
          };
        devShells.default = pkgs.mkShell {
          nativeBuildInputs = with pkgs; [ cmake pkg-config ];
          buildInputs = with pkgs; [
            packages.yasmin
            librealsense
            onnxruntime
            packages.opencv
            boost
            asio
            taskflow
            eigen
            pcl
            packages.cobs-c
            packages.path-planning
            packages.morb_slam
          ];
        };

      });
}
