use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};

// --- Melody Definitions ---

// A rest note (silence)
const REST: u16 = 0;

// Frequencies (in Hz) for the notes in the Super Mario Bros. theme
const NOTE_E5: u16 = 659;
const NOTE_C5: u16 = 523;
const NOTE_G5: u16 = 784;
const NOTE_G4: u16 = 392;

// Frequencies for "Twinkle, Twinkle, Little Star"
const NOTE_C4: u16 = 262;
const NOTE_D4: u16 = 294;
const NOTE_E4: u16 = 330;
const NOTE_F4: u16 = 349;
const NOTE_A4: u16 = 440;

/// Melody for the first part of the Super Mario Bros. theme.
/// Each tuple is (note_frequency, duration_in_beats).
pub const MARIO_MELODY: &[(u16, f32)] = &[
    (NOTE_E5, 1.0), (NOTE_E5, 1.0), (REST, 1.0), (NOTE_E5, 1.0),
    (REST, 1.0), (NOTE_C5, 1.0), (NOTE_E5, 1.0), (REST, 1.0),
    (NOTE_G5, 2.0), (REST, 2.0), (NOTE_G4, 2.0), (REST, 2.0),
];

/// Melody for "Twinkle, Twinkle, Little Star"
pub const TWINKLE_MELODY: &[(u16, f32)] = &[
    (NOTE_C4, 1.0), (NOTE_C4, 1.0), (NOTE_G4, 1.0), (NOTE_G4, 1.0),
    (NOTE_A4, 1.0), (NOTE_A4, 1.0), (NOTE_G4, 2.0), (REST, 0.5),
    (NOTE_F4, 1.0), (NOTE_F4, 1.0), (NOTE_E4, 1.0), (NOTE_E4, 1.0),
    (NOTE_D4, 1.0), (NOTE_D4, 1.0), (NOTE_C4, 2.0),
];


/// A helper function to play a song on a PWM channel.
///
/// * `pwm`: The SimplePwm instance for the timer connected to the buzzer.
/// * `melody`: A slice of (frequency, duration) tuples representing the song.
/// * `tempo_bpm`: The speed of the song in Beats Per Minute (BPM).
pub async fn play_song<'a, T: embassy_stm32::timer::GeneralInstance4Channel>(
    pwm: &mut SimplePwm<'a, T>,
    melody: &[(u16, f32)],
    tempo_bpm: u64,
) {
    // Calculate the duration of a single beat from the tempo
    let beat_duration_ms = 60_000 / tempo_bpm;
    let max_duty = pwm.max_duty_cycle();

    // Set a 50% duty cycle for a nice, loud sound
    pwm.ch1().set_duty_cycle(max_duty / 2);

    for &(note_freq, duration_beats) in melody {
        let note_duration_ms = (duration_beats * beat_duration_ms as f32) as u64;

        if note_freq == REST {
            // For a rest, disable the channel to produce silence
            pwm.ch1().disable();
        } else {
            // For a note, set the correct frequency and enable the channel
            pwm.set_frequency(Hertz::hz(note_freq as u32));
            pwm.ch1().enable();
        }

        // Wait for the duration of the note/rest
        Timer::after(Duration::from_millis(note_duration_ms)).await;
    }

    // Stop the sound after the melody is over
    pwm.ch1().disable();
}