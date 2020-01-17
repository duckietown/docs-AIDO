# Object Detection Dataset {#object-detection-dataset status=beta}

## Download {#object-detection-dataset-download}
The dataset can be downloaded from [here](https://drive.google.com/drive/folders/1cTBoKrXJb0kajBGxhuBxJpbKaotHPX7O). We provide annotations and [sample scripts](https://github.com/saryazdi/Duckietown-Object-Detection-LFV/tree/master/OD_scripts) for loading the annotations.

## Overview {#object-detection-dataset-overview}
This dataset consists of 3 categories: traffic cones, duckies, and duckiebots. All of the dataset images were captured with duckiebot cameras. We use a combination of images from the [Duckietown logs database](http://logs.duckietown.org/) and our own captured logs. Images were captured in different lighting conditions, with different versions of duckiebot models, and on different Duckietown maps. Below are some statistics and visualizations of our dataset:

<br clear="all"/>

<!-- <table>
      <tr><td>Number of images</td><td>1956</td></tr>
      <tr><td>Number of object categories</td><td>3</td></tr>
      <tr><td>Number of objects annotated</td><td>5068</td></tr>
</table> -->

<table style="width: 394px;" border="2" cellpadding="2">
<tbody>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Number of images</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;1956</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Number of object categories</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;3</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 22px;"><strong>Number of objects annotated</strong></td>
<td style="width: 99px; height: 22px;">&nbsp;5068</td>
</tr>
</tbody>
</table>

<figure>
<img src="od_images/annotations.jpg" style='width:35em'/>
</figure>
<br clear="all"/>

## Category Details {#object-detection-dataset-category-details}

### Traffic Cones
<!-- <table>
      <tr><td><b>Category name</b></td><td>cone</td></tr>
      <tr><td><b>Number of instances</b></td><td>372</td></tr>
      <tr><td><b>Category id</b></td><td>1</td></tr>
</table> -->

<table style="width: 394px;" border="2" cellpadding="2">
<tbody>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Category name</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;cone</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Number of instances</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;372</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 22px;"><strong>Category id</strong></td>
<td style="width: 99px; height: 22px;">&nbsp;1</td>
</tr>
</tbody>
</table>

### Duckies
<!-- <table>
      <tr><td>Category name</td><td>duckie</td></tr>
      <tr><td>Number of instances</td><td>2570</td></tr>
      <tr><td>Category id</td><td>2</td></tr>
</table> -->
<table style="width: 394px;" border="2" cellpadding="2">
<tbody>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Category name</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;duckie</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Number of instances</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;2570</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 22px;"><strong>Category id</strong></td>
<td style="width: 99px; height: 22px;">&nbsp;2</td>
</tr>
</tbody>
</table>


### Duckiebots
<!-- <table>
      <tr><td>Category name</td><td>duckiebot</td></tr>
      <tr><td>Number of instances</td><td>2126</td></tr>
      <tr><td>Category id</td><td>3</td></tr>
      <tr><td>Number of old duckiebot instances</td><td>1419</td></tr>
      <tr><td>Number of new duckiebot instances</td><td>707</td></tr>
</table> -->
<table style="width: 394px;" border="2" cellpadding="2">
<tbody>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Category name</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;duckiebot</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 21px;"><strong>Number of instances</strong></td>
<td style="width: 99px; height: 21px;">&nbsp;2126</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 22px;"><strong>Category id</strong></td>
<td style="width: 99px; height: 22px;">&nbsp;3</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 22px;"><strong>Number of old duckiebot instances</strong></td>
<td style="width: 99px; height: 22px;">&nbsp;1419</td>
</tr>
<tr style="height: 22px;">
<td style="width: 304px; height: 22px;"><strong>Number of new duckiebot instances</strong></td>
<td style="width: 99px; height: 22px;">&nbsp;707</td>
</tr>
</tbody>
</table>


## Data Loading Scripts {#object-detection-dataset-data-loading}
We provide some sample scripts for loading in the dataset [here](https://github.com/saryazdi/Duckietown-Object-Detection-LFV/tree/master/OD_scripts).


## Data Collection Procedure {#object-detection-dataset-data-collection}

In this work, we first identify the most prominent objects that we see on the roads of Duckietown: duckies, duckiebots and traffic cones. To begin our data collection procedure, we first identify all useful logs from the [Duckietown logs database](http://logs.duckietown.org/) which contain the objects of interest. We then download and trim these logs so that the videos consist only of frames containing our objects of interest. Finally, we convert our videos to images (frames) while skipping some number of frames between each image to ensure that we get a diverse set of images. 

In these logs, there are videos of older versions of duckiebots with lots of wirings on them. However, new duckiebots are much cleaner with only the battery visible. To ensure robust detections, we needed to capture this intra-class variation. Thus, we collected our own logs containing the new duckiebots. In the final dataset, we have merged old and new duckiebots to ensure that we can detect both variations. 

<figure>
    <img src="od_images/datacollection.gif" style='width:23em'/>
</figure>

## Data Annotation Procedure {#object-detection-dataset-data-annotation}
We used OpenCV's free [CVAT](https://github.com/opencv/cvat) tool to annotate the dataset.

<figure>
<img src="od_images/cvat_annotating.gif" style='width:32em'/>
</figure>
