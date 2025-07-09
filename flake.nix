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
            rev = "4cd3790dba0a86b769093dc167d73a6cc64280cc"; # v3.3.0
            sha256 = "sha256-QCYiqIGETeOhYbVfhIO3HbFxlvIMDalEBT9ygvrOIio=";
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
