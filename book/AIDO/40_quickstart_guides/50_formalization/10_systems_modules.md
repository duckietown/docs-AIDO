
# Systems and modules {#systems-modules status=ready}

## End-to-end solutions and modules

We want to enable end-to-end as well as "modules" solutions.

Consider the following architecture:

<figure id='architecture'>
<figcaption>Original architecture</figcaption>
<img src="connections/architecture.svg" class='diagram'/>
</figure>

## End-to-end solution

An end-to-end solution provides everything.

Imagine drawing a blue diagram around the things that the solution provides

<figure id='example1-group'>
<figcaption>End-to-end</figcaption>
<img src="connections/example1-group.svg" class='diagram'/>
</figure>

Then imagine to compact everything inside:

<figure id='example1-res'>
<figcaption></figcaption>
<img src="connections/example1-res.svg" class='diagram'/>
</figure>



## Almost-end-to-end

Suppose instead that the user wishes to provide only part of the architecture:

<figure id='example2-group'>
<figcaption></figcaption>
<img src="connections/example2-group.svg" class='diagram'/>
</figure>

Then we need to provide the rest; in this case the module "E":

<figure id='example2-res'>
<figcaption></figcaption>
<img src="connections/example2-res.svg" class='diagram'/>
</figure>



## Generic partitition

In general, the user can provide any subset of the nodes.

Draw an arbitrary lasso around the modules:

<figure id='example3-a-group'>
<figcaption></figcaption>
<img src="connections/example3-a-group.svg" class='diagram'/>
</figure>

Then compact it:


<figure id='example3-b'>
<figcaption></figcaption>
<img src="connections/example3-b.svg" class='diagram'/>
</figure>

Then pull it "up", to delineate what we need to provide:

<figure id='example3-c'>
<figcaption></figcaption>
<img src="connections/example3-c.svg" class='diagram'/>
</figure>

## General form

In the general form, this is the layout:

<figure id='sys_general'>
<figcaption></figcaption>
<img src="connections/sys_general.svg" class='diagram'/>
</figure>



<style>
.diagram {
max-width:100%;
margin:1em;
}
</style>
