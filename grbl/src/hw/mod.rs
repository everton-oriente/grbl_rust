// Master file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : Master file for the modular project.
 *  File        : mod.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The master module is responsible about to add or remove modules from the project.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

// Import required crates and modules

mod stepper_pio_dma_multi;


pub(crate) use stepper_pio_dma_multi::*;

