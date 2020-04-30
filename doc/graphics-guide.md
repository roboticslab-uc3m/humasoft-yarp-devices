## A Short Guide to Creating Dynamic Charts with Jupyter Notebook

[Jupyter](https://jupyter.org/) has a beautiful notebook that lets you write and execute code, analyze data, embed content, and share reproducible work.
Therefore, we will use Jupyter Notebook and python language, to process the test data previously stored in `cvs` files using the [pandas](https://pandas.pydata.org/) library, and later visualize them in dynamic graphics thanks to the [plotly](https://plotly.com/) library, being able to expand them, move through them, select the lines that we want to visualize, see the values that make up each point of the lines, export the graph in an image file, etc.
In addition, it allows us to visualize the code blocks that generate these graphs and to be able to modify them as we want. <br>

First of all, if we want to start creating a new graphic or modifying an existing one, we need to [install jupyter-notebook](https://jupyter.readthedocs.io/en/latest/install.html) and start [running](https://jupyter.readthedocs.io/en/latest/running.html#running) it. You can find multiple guides online to learn how to use jupyter. Here we are going to make a small example guide explaining the steps to generate the graph of [Step Input Test using Coupled Control - Inclnation Graph](https://nbviewer.jupyter.org/github/HUMASoft/Data-and-Results/blob/master/demo-results/jupyter-scripts/step_input/step-input-coupled-control.ipynb) from the [csv table](https://github.com/HUMASoft/Data-and-Results/blob/master/demo-results/csv-results/step_input/01-step-input-coupled-control.csv) and publish it.

1. We'll import the necessary packages to process tables and represent dynamic graphs:
```python
import pandas as pd
import plotly.graph_objs as go
from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
```

2. The next step will be to import the data tables stored in `csv` files. These tables are uploaded in humasoft's [Data-and-results](https://github.com/HUMASoft/Data-and-Results) repository. It's important to copy the URL where the raw file is stored. To do this, press on the **RAW** option to access the URL where the file is located. We can see that the url starts like this: https://raw.githubusercontent.com/...
```python
# read CSV
df_coupled = pd.read_csv('https://raw.githubusercontent.com/HUMASoft/Data-and-Results/master/demo-results/csv-results/step_input/01-step-input-coupled-control.csv')
```

3. Next we create the code block referring to the first line that we want to draw on the graph, in this case the target line:
```python
trace_target_inc= go.Scattergl(
    x=df_coupled.index,
    y=df_coupled.target_inclination,
    name = "target inclination",
    mode='lines'
)
```

4. We can add as many lines as we want, in this case, we are interested in also adding the inclination line of the sensor:
```python
trace_sensor_inc= go.Scattergl(
    x=df_coupled.index,
    y=df_coupled.sensor_inclination,
    name = "sensor inclination",
    mode='lines'
)
```
5. We'll make an array that stores all the traces we want to represent.
```python
data = [trace_target_inc, trace_sensor_inc]
```

6. Now, we can add some table properties in the layout field, such as the title, the letter size and format, the name of the 'x' and 'y' axis, the size of the legend and its location, the size of the chart margin, height or width....
All these properties and attributes of each objects that make up the graph are defined at https://plotly.com/python/reference. It's very interesting to look at them to see what can be done.
```python
layout = dict(title= dict(x=0.5, 
                          font={'size':34, 'family':'Overpass'}, 
                          text='Coupled control: Inclination Input Step'),
              xaxis  = dict(title='Samples', autorange=True),
              yaxis  = dict(title='Inclination (degrees)'),
              legend = dict(x=0.82, y=0.98, font={"size": 13}, borderwidth=1),
              margin = dict(t=140, r=20, l=0),
              height = 700
             )
```
7. Finally, we build the graph
```python
fig = go.Figure(data=data, layout=layout)
iplot(fig)
```
8. All the graphics are stored in [Data-and-results](https://github.com/HUMASoft/Data-and-Results) repository in the `jupyter-scripts` section of each experiment. Therefore, we can upload the resulting jupyter-notebook file with the extension `.ipynb`. Github is capable of opening this type of files directly from its web interface, displaying the structure of the python code, but it won't be able to execute the graphics created with plotly. In order to view them online, we'll need to use [nbviewer](https://nbviewer.jupyter.org). We can add the git URL and share it in any place ;)

If you want to follow a more extensive guide on jupyter, I recommend [this one](https://www.dataquest.io/blog/jupyter-notebook-tutorial/) to take a look. Bye!!
