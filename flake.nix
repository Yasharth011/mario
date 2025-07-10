{
  description = "flake for an autonomous rover";

  inputs = {

    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";

    flake-utils.url = "github:numtide/flake-utils";

  };

  outputs = { self, nixpkgs , flake-utils }:
    flake-utils.lib.eachDefaultSystem (system: 
    let pkgs = nixpkgs.legacyPackages.${system};
    in rec {

      packages.yasmin = with pkgs;
        stdenv.mkDerivation {
          name = "yasmin";
          src = fetchFromGitHub {
            owner = "Yasharth011";
            repo = "yasmin_UNROS";
            rev = "f21e30afa122ddc11ff272adf9834e3d18a3b700"; # v3.3.0
            sha256 = "sha256-QRdBoOWUsm9JXgxIq3BUjgwEqJeJpadXnJLlePhCpW8=";
          };
          nativeBuildInputs = [ cmake ];
          configurePhase = ''
	    cd yasmin 
            mkdir build && cd build 
            cmake .. -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out
          '';
	  buildPhase = ''
	    make
	  ''; 
          installPhase = ''
            make install
          '';
        };
      devShells.default =
        pkgs.mkShell { packages = with pkgs; [ cmake packages.yasmin ]; };

    });
}
