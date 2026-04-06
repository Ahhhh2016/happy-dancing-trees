# Project 1: Brush

Please fill this out for Brush only. The project handout can be found [here](https://cs1230.graphics/projects/raster/1).

## Output Comparison
This is how you will demonstrate the functionality of your project.

For each of the rows below, follow the instructions to record a video. Please reference the [Screen Recording Guide](https://cs1230.graphics/docs/screen-recording/) for machine-specific guidance on how to do this.

Once you've recorded everything, navigate to this file in Github, click edit, and either select or drag-and-drop each of your videos into the correct location. This will upload them to GitHub (but not include them in the repo) and automatically embed them into this Markdown file by providing a link. Make sure to double-check that they all show up properly in the preview.

We're **not** looking for your video to **exactly** match the expected output (i.e. draw the exact same shape). Just make sure to follow the instructions and verify that the recording fully demonstrates the functionality of that part of your project.

### Constant Brush
**Instructions:** Draw with the constant brush.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/241d99b5-947a-407c-b454-410534520aad

#### Your Output

<!---
Paste your output on top of this comment!
-->




https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/97444945-1f57-40e4-9935-1db172b2619c




### Linear Brush
**Instructions:** Draw with the linear brush.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/9706fa04-7710-441f-b292-ab010e04dec6

#### Your Output

<!---
Paste your output on top of this comment!
-->



https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/07c8723a-680a-4460-a1d1-06699c08cf35





### Quadratic Brush
**Instructions:** Draw with the quadratic brush.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/c5df5c09-bfe0-4c05-a56e-14609772d675

#### Your Output

<!---
Paste your output on top of this comment!
-->




https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/db6b48da-8157-41ec-bd95-05a420d971e6





### Smudge Brush
**Instructions:** Draw some colors on the canvas and use the smudge brush to smear them together.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/26440b63-2d1c-43fd-95f2-55b74ad3bbed

#### Your Output

<!---
Paste your output on top of this comment!
-->


https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/b53de6b4-60ea-4706-89b5-3ada8f4b6045




### Smudge Brush Change in Alpha
**Instructions:** Draw some colors on the canvas. Use the smudge brush with varying alpha levels (use at least three) and demonstrate that the brush still works the same way each time.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/0b49c7d0-47ca-46d0-af72-48b831dfe7ea

#### Your Output

<!---
Paste your output on top of this comment!
-->


https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/c1757453-bc1d-4b2a-8d0d-e44590ffef6f




### Radius
**Instructions:** Use any brush with at least 3 different values for the radius.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/6f619df6-80cd-4849-8831-6a5aade2a517

#### Your Output

<!---
Paste your output on top of this comment!
-->


https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/710fb407-42a8-4ca5-8879-73bdc5032be6




### Color
**Instructions:** Use any brush to draw red (255, 0, 0), green (0, 255, 0), and blue (0, 0, 255).

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/fd9578ca-e0af-433e-ac9e-b27db2ceebc9

#### Your Output

<!---
Paste your output on top of this comment!
-->


https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/38348da4-bf10-4222-8a49-33f2aefd10ba




### Canvas Edge Behavior
**Instructions:** With any brush, click and draw on the canvas in a place where the mask intersects with the edge. Then, start drawing anywhere on the canvas and drag your mouse off of the edge.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/f5344248-fa5f-4c33-b6df-ff0a45011c7a

#### Your Output

<!---
Paste your output on top of this comment!
-->



https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/f84c4da8-8df5-4ff2-88db-f895ab7be950








### Alpha
**Instructions:** With the constant brush, draw a single dot of red (255, 0, 0) with an alpha of 255. Then, draw over it with a single dot of blue (0, 0, 255) with an alpha of 100. You should get a purpleish color.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/b13d312d-d6d4-4375-aeaa-96174065443b

#### Your Output

<!---
Paste your output on top of this comment!
-->


https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/394da948-3124-4ee2-95e6-1ea16b35a974




### Alpha of Zero
**Instructions:** Choose any brush and demonstrate that it will not draw if the alpha value is zero.

#### Expected Output

https://github.com/BrownCSCI1230/projects_raster_template/assets/77859770/8e48777e-8196-401e-9af6-871abe712146

#### Your Output

<!---
Paste your output on top of this comment!
-->


https://github.com/BrownCSCI1230/projects-raster-Ahhhh2016/assets/23431333/cc2aa1c7-6d8d-461f-9dd5-4724478c62e8




## Design Choices
I basicly followed the algorithms in Algo Answer to design my algorithms. 
For Quadratic bush, I used opacity = distance^2 / R^2 + 2 * distance * R + 1.

## Collaboration/References
Project 1: Brush (Algo Answers)

## Known Bugs
None.

## Extra Credit
I did Spray paint brush.
The code lies in canvas2D.h and canvas2D.cpp. 
  Function initSrand() is used to initialize the random function random seed time.
  Function spray() is used to calculate the random dots that are inside the circle.
  Function drawSprayPaint(int x, int y) is used to draw the dots.


