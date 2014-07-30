
bounding boxes are retrieved from "objectness"
green map of input frame is calculated by summing over retrieved bounding boxes
table boundary is erased with grayBoxes
gBoxes (green boxes) are calculated
cBoxes (connect boxes) are calculated with connected components of gBoxes
bBoxes (blue boxes) are calculated by keeping only some cBoxes
each bBoxes is classified
some bBoxes are oriented
pose is calculated for bBoxes and message is published



notes:

you might need to change the gray boxes
#define DRAW_GRAY

you might need to mess with pose calculation based on orientation
#ifdef DRAW_ORIENTOR

publishing happens here
#define PUBLISH_OBJECTS







