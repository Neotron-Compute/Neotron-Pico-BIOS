{
  description = "Neotron Pico BIOS";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
    flake-utils.lib.eachDefaultSystem
      (system:
        let
          pkgs = (import nixpkgs) {
            inherit system;
            overlays = [ (import rust-overlay) ];
          };
          toolchain = pkgs.pkgsBuildHost.rust-bin.stable.latest.default.override {
            targets = [ "thumbv6m-none-eabi" ];
          };
        in
        {
          devShell = pkgs.mkShell {
            nativeBuildInputs = [
              pkgs.probe-run
              toolchain
            ];
          };
        }
      );
}
