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
use vte::Parser;

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
	let mut statemachine = Parser::new();

	let mut performer = NeotronOutput::new();

	for byte in logo.bytes() {
		statemachine.advance(&mut performer, byte);
	}

	std::fs::write(out.join("logo.bytes"), performer.output).expect("writing logo file");
}

struct NeotronOutput {
	output: Vec<u8>,
	attr: Attr,
	bright: bool,
}

impl NeotronOutput {
	const DEFAULT_ATTR: Attr = Attr::new(
		TextForegroundColour::GREY,
		TextBackgroundColour::BLACK,
		false,
	);

	fn new() -> NeotronOutput {
		NeotronOutput {
			output: Vec::new(),
			attr: Self::DEFAULT_ATTR,
			bright: false,
		}
	}
}

impl vte::Perform for NeotronOutput {
	/// Draw a character to the screen and update states.
	fn print(&mut self, c: char) {
		self.output.push(self.attr.as_u8());
		self.output.push(map_char_to_glyph(c));
	}

	/// A final character has arrived for a CSI sequence
	///
	/// The `ignore` flag indicates that either more than two intermediates arrived
	/// or the number of parameters exceeded the maximum supported length,
	/// and subsequent characters were ignored.
	fn csi_dispatch(
		&mut self,
		params: &vte::Params,
		_intermediates: &[u8],
		ignore: bool,
		action: char,
	) {
		if !ignore && action == 'm' {
			// Colour change
			for param in params.iter() {
				if param.len() != 1 {
					panic!("Only want single value params, got {:?}", param);
				}
				match param[0] {
					1 => {
						self.bright = true;
					}
					// Background
					40 => {
						self.attr.set_bg(TextBackgroundColour::BLACK);
					}
					41 => {
						self.attr.set_bg(TextBackgroundColour::DARK_RED);
					}
					42 => {
						self.attr.set_bg(TextBackgroundColour::DARK_GREEN);
					}
					43 => {
						self.attr.set_bg(TextBackgroundColour::YELLOW);
					}
					44 => {
						self.attr.set_bg(TextBackgroundColour::BLUE);
					}
					45 => {
						self.attr.set_bg(TextBackgroundColour::DARK_MAGENTA);
					}
					46 => {
						self.attr.set_bg(TextBackgroundColour::DARK_CYAN);
					}
					49 => {
						// Default
						self.attr.set_bg(TextBackgroundColour::BLACK);
					}
					// Foreground
					30 => {
						self.attr.set_fg(TextForegroundColour::BLACK);
					}
					31 => {
						self.attr.set_fg(TextForegroundColour::DARK_RED);
					}
					32 => {
						self.attr.set_fg(TextForegroundColour::DARK_GREEN);
					}
					33 => {
						self.attr.set_fg(TextForegroundColour::YELLOW);
					}
					34 => {
						self.attr.set_fg(TextForegroundColour::BLUE);
					}
					35 => {
						self.attr.set_fg(TextForegroundColour::DARK_MAGENTA);
					}
					36 => {
						self.attr.set_fg(TextForegroundColour::DARK_CYAN);
					}
					37 => {
						self.attr.set_fg(TextForegroundColour::GREY);
					}
					0 => {
						self.attr = Self::DEFAULT_ATTR;
						self.bright = false;
					}
					p => {
						panic!("Unsupported ANSI CSI parameter {}", p);
					}
				}
			}
			if self.bright {
				match self.attr.fg() {
					TextForegroundColour::DARK_RED => {
						self.attr.set_fg(TextForegroundColour::BRIGHT_RED);
					}
					TextForegroundColour::DARK_GREEN => {
						self.attr.set_fg(TextForegroundColour::BRIGHT_GREEN);
					}
					TextForegroundColour::YELLOW => {
						self.attr.set_fg(TextForegroundColour::BRIGHT_YELLOW);
					}
					TextForegroundColour::BLUE => {
						self.attr.set_fg(TextForegroundColour::BRIGHT_BLUE);
					}
					TextForegroundColour::DARK_MAGENTA => {
						self.attr.set_fg(TextForegroundColour::BRIGHT_MAGENTA);
					}
					TextForegroundColour::DARK_CYAN => {
						self.attr.set_fg(TextForegroundColour::BRIGHT_CYAN);
					}
					TextForegroundColour::GREY => {
						self.attr.set_fg(TextForegroundColour::WHITE);
					}
					_ => {
						// Do nothing
					}
				}
			}
		}
	}
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
		c => panic!("Unsupported UTF-8 character U+{:04}", c as u32),
	}
}
