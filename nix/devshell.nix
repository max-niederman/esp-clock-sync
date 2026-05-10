{
  config,
  lib,
  pkgs,
  ...
}: {
  devshells.default = let
    inherit (config) packages;
    chosen.rustc =
      if pkgs.stdenv.isLinux
      then packages.bwrap-rustc
      else packages.unsafe-bin-esp-rust;
    chosen.rustdoc =
      if pkgs.stdenv.isLinux
      then packages.bwrap-rustdoc
      else packages.unsafe-bin-esp-rust;
  in
    {config, ...}: {
      name = "esp-rust-nix-sandbox-devshell";

      commands = [
        {package = chosen.rustc;}
        {package = packages.cargo-any-rust;}
        {package = pkgs.rustfmt;}
        {package = pkgs.rust-analyzer;}
        {package = pkgs.clippy;}
        {package = pkgs.espflash;}
        {package = pkgs.picocom;}
        {
          name = "host-cargo";
          help = "cargo invocation that uses the regular nixpkgs rustc (host-target builds only)";
          command = ''
            exec env \
              -u RUSTC \
              -u RUSTFLAGS \
              -u CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS \
              -u CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER \
              PATH="${pkgs.rustc}/bin:$PATH" \
              RUSTFLAGS="" \
              CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS="" \
              CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER=cc \
              ${pkgs.cargo}/bin/cargo "$@"
          '';
        }
      ];

      env = [
        {
          name = "RUSTC";
          value = lib.getExe chosen.rustc;
        }
        {
          name = "CARGO_HOME";
          # We need it to be under $PRJ_ROOT, so that the sandboxed `rustc` has
          # access to it.
          eval = ''"$PRJ_ROOT"/target/cargo-home'';
        }
        {
          name = "RUST_SRC_PATH";
          value = "${packages.esp-rust-src}/lib/rustlib/src/rust/library";
        }
        {
          name = "ESPFLASH_SKIP_UPDATE_CHECK";
          value = "true";
        }
      ];

      devshell = {
        packages = [
          pkgs.unixtools.xxd
          chosen.rustdoc
        ];

        motd = ''

          {202}🔨 Welcome to ${config.name}{reset}

          Untrusted binary blobs (pre-built Rust and GCC compilers) are run in a strict Bubblewrap
          ({bold}bwrap{reset}) sandbox with access only to {bold}$PRJ_ROOT{reset}.

          The other tools (Cargo, espflash, etc.) are source-based and come from regular Nixpkgs.
          $(menu)

          ESP32 firmware (sandboxed esp-fork rustc):
            • {bold}cd crates/clock-sync-firmware && cargo build --release --features esp32{reset}
            • {bold}cd crates/clock-sync-test     && cargo build --release --features esp32{reset}
            • {bold}cd crates/clock-sync-firmware && cargo run   --release --features esp32{reset}  (flash + monitor)

          Host (Linux) binaries (regular nixpkgs rustc — uses {bold}host-cargo{reset}):
            • {bold}cd crates/clock-sync-server && host-cargo build --release{reset}
            • {bold}cd crates/skew-meter        && host-cargo build --release{reset}

          Serial monitor (alternative):
            • {bold}picocom --baud=115200 --imap lfcrlf /dev/ttyUSB0{reset}
        '';

        startup.verify-bwrap.text = lib.getExe packages.verify-bwrap;
      };
    };
}
