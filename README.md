# Urho3D Navigational Polyflag editor

Description
-----
Navigational Polyflag editor for Urho3D.  
Note that this is still **WIP**. Eventually, all the navmesh configuration settings that's written for this type of nav samples should become data driven and parsed from XML file, and make navfile a resource class.  

#### Demo
You can edit polyflags on tiles and have characters traverse specific tiles.  

Character colors:
* white - can travers on yellow tiles
* blue - can traverse on blue + yellow
* green - can traverse on all, except on holes

Implementation Notes
-----
By default, all drawable and collision objects are disabled as navigatable objects. If you ever created a game level or scene, you'll notice that only a very small fraction of objects are actually navigatable compared to all the renderable objects in a game.  

To enable objects as navigatable, set the **"Nav Flag"** to a non-zero value in either the class attribute inherited from Drawable or the CollisionShape class. It's not a *bool* variable, but *unsigned* as I was experimenting how I can pass this value to Navmesh and have it automatically set the polyflag values, but because there was so much overlap in tiles in most cases, I'd remove that feature for now.  

There is a navfile save option in the editor, and the loading happens automatically if a *navfile*.nav file is found at the start of the program.  Having this ".nav" extention causes Urho3D editor to display an error message saying something like "unable to parse XML file."  To prevent this annoying error message when opening the editor, I've added the ".nav" extension info to the EditorResourceBrowser.as file. It doesn't do anymore than that atm.


Screenshot
-----
![alt tag](https://github.com/Lumak/Urho3D-NavPolyFlag-Editor/blob/master/screenshot/Screenshot.png)


To Build
-----
To build it, unzip/drop the repository into your Urho3D/ folder and build it the same way as you'd build the default Samples that come with Urho3D.
**Built with Urho3D 1.7 tag.**

License
-----------------------------------------------------------------------------------
The MIT License (MIT)







