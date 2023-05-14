{
  description = "Neotron Pico BIOS";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    neotron-os-input.url = "github:Detegr/Neotron-OS/flake";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-utils.follows = "flake-utils";
      };
    };
    crane = {
      url = "github:ipetkov/crane";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-utils.follows = "flake-utils";
        rust-overlay.follows = "rust-overlay";
      };
    };
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay, crane, neotron-os-input }:
    let
      # List of systems this flake.nix has been tested to work with
      systems = [ "x86_64-linux" ];
      pkgsForSystem = system: nixpkgs.legacyPackages.${system}.appendOverlays [
        rust-overlay.overlays.default
      ];
    in
    flake-utils.lib.eachSystem systems
      (system:
        let
          pkgs = pkgsForSystem system;
          toolchain = pkgs.rust-bin.stable.latest.default.override {
            targets = [ "thumbv6m-none-eabi" ];
          };
          craneLib = (crane.mkLib pkgs).overrideToolchain toolchain;
          neotron-os = neotron-os-input.packages.${system}.neotron-os-thumbv6m;
          neotron-os-bin = "thumbv6m-none-eabi-flash1002-libneotron_os.bin";

          bios-deps = craneLib.buildDepsOnly {
            pname = "neotron-pico-bios";
            cargoExtraArgs = "--target=thumbv6m-none-eabi";
            doCheck = false;
            cargoToml = ./Cargo.toml;
            cargoLock = ./Cargo.lock;
            dummySrc = craneLib.mkDummySrc {
              src = craneLib.path ./.;
              extraDummyScript = ''
                rm -rf $out/.cargo
              '';
            };
          };
          bios = craneLib.buildPackage {
            pname = "neotron-pico-bios";
            cargoArtifacts = bios-deps;
            src = with pkgs.lib;
              let keep = suffixes: path: type: any (s: hasSuffix s path) suffixes;
              in
              cleanSourceWith {
                src = craneLib.path ./.;
                filter = path: type: any id (map (f: f path type) [
                  craneLib.filterCargoSources
                  (keep [ ".x" ])
                ]);
              };
            doCheck = false;
            cargoExtraArgs = "--target=thumbv6m-none-eabi";
            nativeBuildInputs = [
              pkgs.makeWrapper
              pkgs.probe-run
            ];
            buildInputs = [
              neotron-os
            ];
            postPatch = ''
              substituteInPlace src/main.rs --replace ${neotron-os-bin} ${neotron-os}/bin/${neotron-os-bin}
            '';

            postInstall = ''
              makeWrapper ${pkgs.probe-run}/bin/probe-run $out/bin/flash-neotron-pico-bios \
                --set DEFMT_LOG debug \
                --add-flags "--chip RP2040 --measure-stack $out/bin/neotron-pico-bios"
            '';
          };
        in
        rec
        {
          packages = {
            neotron-pico-bios = bios;
            default = packages.neotron-pico-bios;
          };

          defaultApp = apps.neotron-pico-bios;
          apps.neotron-pico-bios = flake-utils.lib.mkApp {
            drv = bios;
            exePath = "/bin/flash-neotron-pico-bios";
          };

          devShell = pkgs.mkShell {
            inputsFrom = builtins.attrValues self.packages.${system};
          };
        }
      );
}
