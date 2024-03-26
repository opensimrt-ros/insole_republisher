# Republisher

## Original Intention

In the beginning I wanted to republish insole data with a fixed rate with this and came up with a convoluted solution which was actually inserting extra elements when there was a lack of info after a burst. I believe now that this is not a good solution to the problem, since you need to go backwards in time and keep track of separate streams, the publication stream and the incoming stream to update it accordingly with the newest data. This is still a good solution for the edge case where the buffer in the downstream part of the pipeline is too short and you need to show some information. A sequence of large burst lags would show at least some info for the downstream nodes (if you do this the "simple" way I was doing before you run into a problem where you never have information that is new enough to show, so you only downstream the last known less than the buffer delay valid sample, so it looks like it is stopped). 

If this sounds complicated is because it was really complicated. And to make it work the way i intended it would be even harder. I wanted to make the republisher class a generic latest possible limited buffer size publisher that maybe would even filter the values for you, but debugging templates can be tricky and i couldnt come up with a class structure that was adequate for this problem. 

This is the intention of this package. 

But right now it will not do any of that anymore. 


## Current very simplified functionality


Right now this package will only republish insole messages and probably filter COPs and total forces. It will NOT be generic, I don't have the time for this. 

- Republisher: This guy gets the messages from the insoles that are packed all together ()
