# Inspiration

As a group of current and ex-musicians, we all have experience trying to improve our skills while trying not to make it everyone else's business. But with the inescapable loud noise of an instrument, it's really difficult and embarrassing! So we thought: what if our playing could be ours alone? To revise, improve, and practice as much as we want without being yelled at? We wanted to jam silently, so we created Silent Jam!

Check out our demo video, filmed and edited by Arjita Chaudhary: https://www.youtube.com/watch?v=H2Lhavi_kFE

<img width="4284" height="5712" alt="IMG_6144" src="https://github.com/user-attachments/assets/e2a92563-6f67-4a0f-bf24-635e73331437" />

*A picture of our beautiful creation!*

# What it does

It's a smart mute that is inserted into the bell of a brass instrument. Inspired by traditional brass mutes, which alter the sound of the instrument, our smart mute both muffles the instrument's sound to a negligible degree and allows the user to listen to their playing through their headphones, creating an embarrassment-free musical experience.

<img width="4284" height="5712" alt="IMG_6145" src="https://github.com/user-attachments/assets/8a5fc929-e1c1-4528-b1f1-b1527dc1543d" />

*See our mute in action!*

# How we built it

The work began with creating a Python algorithm that takes audio input, assesses its pitch and decibel levels, and then outputs a synthesized version. We simultaneously designed a 3D-printed mute design resembling a traditional brass mute but with increased audio muffling ability. Then, we reiterated different versions of the code, translating it into C++ for improved efficiency and sound clarity. Finally, we put together the circuitry for the smart mute, connecting the algorithm with a Raspberry Pi and a physical button, allowing the user to record their listening and save it as a file to their computer.

# Challenges we ran into

## Latency: 
There was a noticeable delay between the user playing the note and the sound actually being heard through the headphones. We translated the code from Python to C++ to decrease the delay. Tone inaccuracies: It was difficult to sync the audio playback in the user's headphones with their playing. We revised the design of the smart mute to change the placement of the USB mic to be in the perfect zone to prevent tone distortion.

# Accomplishments that we're proud of

## Audio input:
synthesized output & recorder algorithm: We're super proud of overcoming the latency and tonal challenges we initially faced. The result was a super cool synthesizer algorithm that both generated an accurate output and recorded the generated output for the user to reference. Smart mute's muffle-ability: Our mute's muffle-ability far exceeds our initial expectations, creating a barely-audible yet comfortable playing experience for users with minimal air pushback and auditory discomfort.
What we learned

## Science Rules!
Deeper understanding of applied physics concepts: While creating prototypes, it was critical to meet the intricate balance between perfectly muffled and minimal air pushback for the player. This taught us a great deal about the applications of physics concepts, such as sound waves and how they travel. NFCs: Though we didn't include NFCs in the final prototype, we learned a lot about implementing the technology into hardware projects and are enthused to incorporate it into future builds!

<img width="3300" height="2550" alt="Trumpet Mute Channel Piece" src="https://github.com/user-attachments/assets/cf860bc6-fb89-470d-8303-1b324c9609db" />

<img width="3300" height="2550" alt="Trumpet Adaptor" src="https://github.com/user-attachments/assets/614aa919-782d-47bc-8fc8-f23bdb7dd3eb" />

<img width="3300" height="2550" alt="Trumpet Mute" src="https://github.com/user-attachments/assets/d503329c-ce3b-4037-bd52-928fed73eb84" />

*Some pictures of the 3D models that became our mute! Shoutout Eric Korczak for being the OnShape GOAT!*

# What's next for Silent Jam

## Collaborative jamming
Expanding the algorithm to accommodate several Silent Jam mute users to play in the same environment, and receive each other's auditory feedback simultaneously. Modular mutes: Expanding the mute design to be used for woodwind instruments (which are more challenging to accommodate, given their multiple wind passageways). 
