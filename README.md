# Kelaynak84

Kelaynak 84 is a *combat flight simulation* and *trading* game that takes inspiration from flight simulators such as [**FlightGear**](https://www.flightgear.org/), [**Microsoft Flight Simulator**](https://en.wikipedia.org/wiki/Microsoft_Flight_Simulator_1.0), 
[**Digital Combat Simulator**](https://www.digitalcombatsimulator.com/en/), [**Falcon 4**](https://www.microprose.com/games/falcon-4/) and the influential 1984 classic space trading game [**Elite**](https://en.wikipedia.org/wiki/Elite_(video_game)).
It mixes up the gameplay elements of Elite with a classical flight simulator, and just like Elite did, it uses wireframe 3D graphics.

![kelaynak_ss1](https://github.com/arda-guler/Kelaynak84/assets/80536083/de3b05ec-e5c2-4531-b259-502aeb2e6d94)

## Gameplay
### Main Game (main.py)
In the main game, you start with a poor-performance aircraft with little money. You need to trade commodities between cities and buy better aircraft components to suit your needs. A larger, bulkier airframe will be handy for that!

As it is the case with Elite, mean pirates will sometimes appear and try to shoot you down. You will have your RWR (Radar Warning Receiver) alerts and all-aspect guided rockets to defend yourself. A more maneuverable airframe will come in handy here!

### Quick Dogfight (quick_dogfight.py)
This will start you in the air and will almost immediately start a dogfight. Once you defeat one opponent, the other will appear right away. How many waves can you survive?

### Auto Dogfight (auto_dogfight.py)
This will remove all manual controls and put your autopilot controlled plane against an opponent. If you think you can program a better autopilot, edit auto_dogfight.py and put your algorithms to test!

## The Game Universe
Currently, there are three fictional cities in the game world; Yazılıkaya, Numakawa, Meadowview. Each city houses two aircraft manufacturers, which you can buy components from. The prices of common commodities change according to the predetermined economic activities of the cities, like in Elite.

### Yazılıkaya
A moderately sized agricultural city.

Home to **Ousteem Aerospace**, which specializes in lightweight airframes. 

Home to **AG Power Works**, which specializes in heavy-duty propulsion.

Developer's note: If you want to make a good fighter aircraft, you buy components from here.

### Numakawa
A small industrial city.

Home to **Matsuboshi**, which specializes in heavy-lift airframes.

Home to **Kobesaki**, which specializes in fuel-efficient propulsion.

Developer's note: If you want to make a good freighter, buy components from here.

### Meadowview
A large industrial metropolis.

Home to **Logheat Mardin**, which specializes in serial production of airframes.

Home to **Special Electricity**, which specializes in serial production of propulsion systems.

Developer's note: If you want a cheap aircraft, buy components from here.

## Default Controls
**W, A, S, D, Q, E**: Move control surfaces

**Shift + W**, **Shift + S**: Adjust elevator trim

**U, I, O, J, K, L**: Rotate camera

**T, G**: Start & stop APU (you may need this to start your engine)

**Z, X**: Throttle up & down

You have to use the console window to do trading in cities.

## Music License
Background music is from [Plutonia MIDI Pack](https://www.doomworld.com/idgames/music/plutmidi). License is included in [plutmidi.txt](https://github.com/arda-guler/Kelaynak84/blob/master/plutmidi.txt).


