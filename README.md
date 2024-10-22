# Secure UART Lock System
Final Project for Embedded Systems Course

Overview:
The UART Access Security System is an embedded system project designed to provide password-protected access control using UART communication and LED indicators. This system offers real-time feedback for password entry, error handling, and a reaction-based game mode, enhancing both security and user interaction.

Features:
 - Password-Protected Access: The system requires users to enter a 3-character password via UART. LEDs provide visual feedback for each correct or incorrect input, with flags controlling the state of the device.
- LED Indicators & Timeout: Each password state corresponds to a different LED flash rate, with a timeout mechanism that locks the system after 50 flashes.
- Visual ADC Data Representation: A bar generator visually represents ADC values, displaying real-time analog sensor data via UART.
- Reaction Game Mode: A reaction-based game is initiated by pressing a push button, allowing users to choose between two difficulty modes. Baud rates adjust dynamically based on difficulty, maintaining accurate UART communication.
- Password Reset Functionality: The system includes a password reset feature, which can be activated by pressing a designated button, enabling users to update the password securely.

How It Works:
1. Initial State: The system prompts the user to enter a password character over UART. If the character is correct, a flag is set, and the system moves to the next character.
2. Incorrect Password: If an incorrect character is entered, an error is triggered, halting the system until a button press resets it.
3. Password Verification: Once all characters are correctly entered, the system transitions to the "unlocked" state, allowing access to other features like the ADC bar generator and reaction game.
4.  - Password Reset: Pressing a specific button enters password reset mode, allowing the user to input a new password via UART. (or)
    - Reaction Game: Users can initiate the reaction game by pressing another button, where the game mode and clock settings dynamically adjust based on the selected difficulty.

Technical Details:
- UART Communication: UART is used for sending and receiving data between the system and the user interface.
- Flags and States: The system relies on flag bits to track the state of the password entry, LED flashes, and game interactions.
- Error Handling: Incorrect password input and communication errors trigger feedback mechanisms that ensure proper system behavior.
- Timers and Delays: Timeout mechanisms ensure the system locks after a certain number of flashes or input cycles, maintaining security and responsiveness.

Possible Future Improvements:
- Add multi-user support for storing and validating multiple passwords.
- Implement a graphical user interface.
