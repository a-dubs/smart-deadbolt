# Custom Smart Deadbolt 

Brainstormed, designed, built, and programmed a smart deadbolt from scratch

<br>

## Summary
- Attaches to my backdoor’s OEM deadbolt and turns the deadbolt lever to lock and unlock door
- Mounts to the door without any adhesive or mounting hardware that could potentially damage the door
- Leverages radio modules for communication with smart home controller 
- Powered by an Arduino Pro Mini 5v that drives a high torque TT motor 
- The enclosure and all internal parts were designed from scratch in Autodesk Inventor and 3D printed by yours truly
- Custom modular design allows for rapid prototyping and iterative design 
- The deadbolt lever has a 3D-printed adapter that snaps around it (no adhesive necesssary) to convert it into one big gear 
- This gear is then turned by the TT motor through a smaller gear with half the teeth which  
- Limit switch senses when door is closed or open - useful for functionality such as preventing deadbolt from being extended when trying to shut door
- Potentiometer sitting opposite the motor shaft provides two-fold functionality: 
  - Acts as an axle of sorts for the gear driven by the TT motor
  - Is a highly precise, non-volatile, axle "encoder" that allows for knowing the position of the deadbolt gear, and can thus infer when the deadbolt is turned into the unlocked or locked position
- Seamlessly integrates with my custom smart home ecosystem
- Functionality provided by Smart Home:
  - Automatically locks deadbolt after a minute if it is left unlocked
  - Remote monitoring of whether door is ajar/open
  - Remote monitoring and control of the deadbolt from anywhere, enabling the ability to let guests into home remotely
  - Function that prevents deadbolt from locking for any reason - useful for when you want to step outside quickly without needing your keys or a smarthpone 
<br>

## Image Gallery

### Assembly of 3D Printed Components
![assembly of 3d printed components](https://github.com/a-dubs/smart-deadbolt/blob/master/image_gallery/full_assembly_1.jpg)

<br>

### Assembly of 3D Printed Components
![assembly of 3d printed components](https://github.com/a-dubs/smart-deadbolt/blob/master/image_gallery/full_assembly_2.jpg)

<br>

<!-- ### View w/ Clear Top
![view with clear top](https://github.com/a-dubs/smart-deadbolt/blob/master/image_gallery/full_assembly_clear_roof_1.jpg)

<br>
 -->
### View w/ Clear Sides
![view with clear sides](https://github.com/a-dubs/smart-deadbolt/blob/master/image_gallery/full_assembly_clear_sides_1.jpg)

<br>

### Exploded View
![exploded view](https://github.com/a-dubs/smart-deadbolt/blob/master/image_gallery/full_assembly_expanded_1.jpg)

<br>

### Exploded View w/ Clear Sides & Bottom
![exploded view with clears sides and bottom](https://github.com/a-dubs/smart-deadbolt/blob/master/image_gallery/full_assembly_expanded_2.jpg)

<br>


<br>

## Project Metadata

**Project Status** : Active  
**Project Progress** : In Progress
**Project Dates** : Oct '21 - Present

<!-- portfolio.alecwarren.com position priority = 9 (-1 is lowest, 0 is default, 10 is highest) -->
