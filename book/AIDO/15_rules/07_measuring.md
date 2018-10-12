# Measuring performance {#measuring-performance status=ready}

Measuring performance in robotics is less clear cut and more multidimensional than traditionally encountered in machine learning settings. Nonetheless, to achieve reliable performance estimates we assess submitted code on several *episodes* with different initial settings and compute statistics on the outcomes. We denote $\objective$ to be an objective or cost function to optimize, which we evaluate for every experiment. In the following formalization, objectives are assumed to be minimized.

In the following we summarize the objectives used to quantify how well an embodied task is completed. We will produce scores in three different categories:

* [*Performance objective*](#performance)

* [*Traffic law objective*](#traffic_laws)

* [*Comfort objective*](#comfort).

Note that the these objectives are not merged into one single number.
