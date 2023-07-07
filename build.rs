//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use neotron_common_bios::video::{Attr, TextBackgroundColour, TextForegroundColour};

fn main() {
	// Put `memory.x` in our output directory and ensure it's
	// on the linker search path.
	let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
	File::create(out.join("memory.x"))
		.unwrap()
		.write_all(include_bytes!("memory.x"))
		.unwrap();
	println!("cargo:rustc-link-search={}", out.display());

	// By default, Cargo will re-run a build script whenever
	// any file in the project changes. By specifying `memory.x`
	// here, we ensure the build script is only re-run when
	// `memory.x` is changed.
	println!("cargo:rerun-if-changed=memory.x");

	// Generate a file containing the firmware version
	let mut output;
	if let Ok(version_output) = std::process::Command::new("git")
		.current_dir(env::var_os("CARGO_MANIFEST_DIR").unwrap())
		.args(["describe", "--tags", "--dirty"])
		.output()
	{
		println!(
			"Version is {:?}",
			std::str::from_utf8(&version_output.stdout)
		);
		println!("Error is {:?}", std::str::from_utf8(&version_output.stderr));
		assert!(version_output.status.success());

		// Remove the trailing newline
		output = version_output.stdout;
		output.pop();
	} else {
		output = String::from(env!("CARGO_PKG_VERSION")).into_bytes();
	}

	// Add a null
	output.push(0);

	// Write the file
	std::fs::write(out.join("version.txt"), output).expect("writing version file");

	let logo = std::fs::read_to_string("src/logo.ansi").expect("Can't open logo file");
	let mut output = Vec::new();
	let mut iter = logo.chars();
	let mut attr = Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::BLACK,
		false,
	);
	let mut width = 0;
	loop {
		let Some(ch) = iter.next() else {
			break;
		};
		if ch == '\u{001b}' {
			// ANSI escape
			if iter.next() == Some('[') {
				// CSI
				let mut bright = false;
				let mut accumulator = 0;
				loop {
					let next_ch = iter.next();
					match (next_ch, accumulator) {
						// Bright
						(Some(';' | 'm'), 1) => {
							bright = true;
							accumulator = 0;
						}
						// Background
						(Some(';' | 'm'), 40) => {
							attr.set_bg(TextBackgroundColour::BLACK);
							accumulator = 0;
						}
						(Some(';' | 'm'), 41) => {
							attr.set_bg(TextBackgroundColour::DARK_RED);
							accumulator = 0;
						}
						(Some(';' | 'm'), 42) => {
							attr.set_bg(TextBackgroundColour::DARK_GREEN);
							accumulator = 0;
						}
						(Some(';' | 'm'), 43) => {
							attr.set_bg(TextBackgroundColour::YELLOW);
							accumulator = 0;
						}
						(Some(';' | 'm'), 44) => {
							attr.set_bg(TextBackgroundColour::BLUE);
							accumulator = 0;
						}
						(Some(';' | 'm'), 45) => {
							attr.set_bg(TextBackgroundColour::DARK_MAGENTA);
							accumulator = 0;
						}
						(Some(';' | 'm'), 46) => {
							attr.set_bg(TextBackgroundColour::DARK_CYAN);
							accumulator = 0;
						}
						(Some(';' | 'm'), 49) => {
							// Default
							attr.set_bg(TextBackgroundColour::BLACK);
							accumulator = 0;
						}
						// Foreground
						(Some(';' | 'm'), 30) => {
							attr.set_fg(TextForegroundColour::BLACK);
							accumulator = 0;
						}
						(Some(';' | 'm'), 31) => {
							attr.set_fg(TextForegroundColour::DARK_RED);
							accumulator = 0;
						}
						(Some(';' | 'm'), 32) => {
							attr.set_fg(TextForegroundColour::DARK_GREEN);
							accumulator = 0;
						}
						(Some(';' | 'm'), 33) => {
							attr.set_fg(TextForegroundColour::YELLOW);
							accumulator = 0;
						}
						(Some(';' | 'm'), 34) => {
							attr.set_fg(TextForegroundColour::BLUE);
							accumulator = 0;
						}
						(Some(';' | 'm'), 35) => {
							attr.set_fg(TextForegroundColour::DARK_MAGENTA);
							accumulator = 0;
						}
						(Some(';' | 'm'), 36) => {
							attr.set_fg(TextForegroundColour::DARK_CYAN);
							accumulator = 0;
						}
						(Some(';' | 'm'), 37) => {
							attr.set_fg(TextForegroundColour::GREY);
							accumulator = 0;
						}

						(Some(';' | 'm'), 0) => {
							attr.set_fg(TextForegroundColour::GREY);
							attr.set_bg(TextBackgroundColour::BLACK);
							bright = false;
							accumulator = 0;
						}

						(Some('0'), _) => {
							accumulator *= 10;
						}
						(Some('1'), _) => {
							accumulator *= 10;
							accumulator += 1;
						}
						(Some('2'), _) => {
							accumulator *= 10;
							accumulator += 2;
						}
						(Some('3'), _) => {
							accumulator *= 10;
							accumulator += 3;
						}
						(Some('4'), _) => {
							accumulator *= 10;
							accumulator += 4;
						}
						(Some('5'), _) => {
							accumulator *= 10;
							accumulator += 5;
						}
						(Some('6'), _) => {
							accumulator *= 10;
							accumulator += 6;
						}
						(Some('7'), _) => {
							accumulator *= 10;
							accumulator += 7;
						}
						(Some('8'), _) => {
							accumulator *= 10;
							accumulator += 8;
						}
						(Some('9'), _) => {
							accumulator *= 10;
							accumulator += 9;
						}
						(code, acc) => {
							panic!(
								"Invalid ANSI sequence detected: code={:?}, acc={:?}",
								code, acc
							);
						}
					}
					if next_ch == Some('m') {
						// sequence is finished
						if bright {
							match attr.fg() {
								TextForegroundColour::DARK_RED => {
									attr.set_fg(TextForegroundColour::BRIGHT_RED);
								}
								TextForegroundColour::DARK_GREEN => {
									attr.set_fg(TextForegroundColour::BRIGHT_GREEN);
								}
								TextForegroundColour::YELLOW => {
									attr.set_fg(TextForegroundColour::BRIGHT_YELLOW);
								}
								TextForegroundColour::BLUE => {
									attr.set_fg(TextForegroundColour::BRIGHT_BLUE);
								}
								TextForegroundColour::DARK_MAGENTA => {
									attr.set_fg(TextForegroundColour::BRIGHT_MAGENTA);
								}
								TextForegroundColour::DARK_CYAN => {
									attr.set_fg(TextForegroundColour::BRIGHT_CYAN);
								}
								TextForegroundColour::GREY => {
									attr.set_fg(TextForegroundColour::WHITE);
								}
								_ => {
									// Do nothing
								}
							}
						}
						break;
					}
				}
			}
		} else if ch == '\n' {
			if width < 80 {
				// Only emit a new-line if we are not at the end of the line already
				output.push(attr.as_u8());
				output.push(b'\n');
			}
			width = 0;
		} else {
			width += 1;
			output.push(attr.as_u8());
			output.push(map_char_to_glyph(ch));
		};
	}
	std::fs::write(out.join("logo.bytes"), output).expect("writing logo file");
}

/// Convert a Unicode Scalar Value to a font glyph.
///
/// Zero-width and modifier Unicode Scalar Values (e.g. `U+0301 COMBINING,
/// ACCENT`) are not supported. Normalise your Unicode before calling
/// this function.
fn map_char_to_glyph(input: char) -> u8 {
	// This fixed table only works for the default font. When we support
	// changing font, we will need to plug-in a different table for each font.
	match input {
		'\u{0000}'..='\u{007F}' => input as u8,
		'\u{00A0}' => 255, // NBSP
		'\u{00A1}' => 173, // ¡
		'\u{00A2}' => 189, // ¢
		'\u{00A3}' => 156, // £
		'\u{00A4}' => 207, // ¤
		'\u{00A5}' => 190, // ¥
		'\u{00A6}' => 221, // ¦
		'\u{00A7}' => 245, // §
		'\u{00A8}' => 249, // ¨
		'\u{00A9}' => 184, // ©
		'\u{00AA}' => 166, // ª
		'\u{00AB}' => 174, // «
		'\u{00AC}' => 170, // ¬
		'\u{00AD}' => 240, // SHY
		'\u{00AE}' => 169, // ®
		'\u{00AF}' => 238, // ¯
		'\u{00B0}' => 248, // °
		'\u{00B1}' => 241, // ±
		'\u{00B2}' => 253, // ²
		'\u{00B3}' => 252, // ³
		'\u{00B4}' => 239, // ´
		'\u{00B5}' => 230, // µ
		'\u{00B6}' => 244, // ¶
		'\u{00B7}' => 250, // ·
		'\u{00B8}' => 247, // ¸
		'\u{00B9}' => 251, // ¹
		'\u{00BA}' => 167, // º
		'\u{00BB}' => 175, // »
		'\u{00BC}' => 172, // ¼
		'\u{00BD}' => 171, // ½
		'\u{00BE}' => 243, // ¾
		'\u{00BF}' => 168, // ¿
		'\u{00C0}' => 183, // À
		'\u{00C1}' => 181, // Á
		'\u{00C2}' => 182, // Â
		'\u{00C3}' => 199, // Ã
		'\u{00C4}' => 142, // Ä
		'\u{00C5}' => 143, // Å
		'\u{00C6}' => 146, // Æ
		'\u{00C7}' => 128, // Ç
		'\u{00C8}' => 212, // È
		'\u{00C9}' => 144, // É
		'\u{00CA}' => 210, // Ê
		'\u{00CB}' => 211, // Ë
		'\u{00CC}' => 222, // Ì
		'\u{00CD}' => 214, // Í
		'\u{00CE}' => 215, // Î
		'\u{00CF}' => 216, // Ï
		'\u{00D0}' => 209, // Ð
		'\u{00D1}' => 165, // Ñ
		'\u{00D2}' => 227, // Ò
		'\u{00D3}' => 224, // Ó
		'\u{00D4}' => 226, // Ô
		'\u{00D5}' => 229, // Õ
		'\u{00D6}' => 153, // Ö
		'\u{00D7}' => 158, // ×
		'\u{00D8}' => 157, // Ø
		'\u{00D9}' => 235, // Ù
		'\u{00DA}' => 233, // Ú
		'\u{00DB}' => 234, // Û
		'\u{00DC}' => 154, // Ü
		'\u{00DD}' => 237, // Ý
		'\u{00DE}' => 232, // Þ
		'\u{00DF}' => 225, // ß
		'\u{00E0}' => 133, // à
		'\u{00E1}' => 160, // á
		'\u{00E2}' => 131, // â
		'\u{00E3}' => 198, // ã
		'\u{00E4}' => 132, // ä
		'\u{00E5}' => 134, // å
		'\u{00E6}' => 145, // æ
		'\u{00E7}' => 135, // ç
		'\u{00E8}' => 138, // è
		'\u{00E9}' => 130, // é
		'\u{00EA}' => 136, // ê
		'\u{00EB}' => 137, // ë
		'\u{00EC}' => 141, // ì
		'\u{00ED}' => 161, // í
		'\u{00EE}' => 140, // î
		'\u{00EF}' => 139, // ï
		'\u{00F0}' => 208, // ð
		'\u{00F1}' => 164, // ñ
		'\u{00F2}' => 149, // ò
		'\u{00F3}' => 162, // ó
		'\u{00F4}' => 147, // ô
		'\u{00F5}' => 228, // õ
		'\u{00F6}' => 148, // ö
		'\u{00F7}' => 246, // ÷
		'\u{00F8}' => 155, // ø
		'\u{00F9}' => 151, // ù
		'\u{00FA}' => 163, // ú
		'\u{00FB}' => 150, // û
		'\u{00FC}' => 129, // ü
		'\u{00FD}' => 236, // ý
		'\u{00FE}' => 231, // þ
		'\u{00FF}' => 152, // ÿ
		'\u{0131}' => 213, // ı
		'\u{0192}' => 159, // ƒ
		'\u{2017}' => 242, // ‗
		'\u{2500}' => 196, // ─
		'\u{2502}' => 179, // │
		'\u{250C}' => 218, // ┌
		'\u{2510}' => 191, // ┐
		'\u{2514}' => 192, // └
		'\u{2518}' => 217, // ┘
		'\u{251C}' => 195, // ├
		'\u{2524}' => 180, // ┤
		'\u{252C}' => 194, // ┬
		'\u{2534}' => 193, // ┴
		'\u{253C}' => 197, // ┼
		'\u{2550}' => 205, // ═
		'\u{2551}' => 186, // ║
		'\u{2554}' => 201, // ╔
		'\u{2557}' => 187, // ╗
		'\u{255A}' => 200, // ╚
		'\u{255D}' => 188, // ╝
		'\u{2560}' => 204, // ╠
		'\u{2563}' => 185, // ╣
		'\u{2566}' => 203, // ╦
		'\u{2569}' => 202, // ╩
		'\u{256C}' => 206, // ╬
		'\u{2580}' => 223, // ▀
		'\u{2584}' => 220, // ▄
		'\u{2588}' => 219, // █
		'\u{2591}' => 176, // ░
		'\u{2592}' => 177, // ▒
		'\u{2593}' => 178, // ▓
		'\u{25A0}' => 254, // ■
		_ => b'?',
	}
}
