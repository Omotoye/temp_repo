from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

# Creating a figure object instance
fig = Figure()

# Putting the figure in a figure Canvas
Canvas = FigureCanvas(fig)

x = np.random.randn(10000)

## Adding a subplot to the figure, which then returns the axes object 
# of the subplot
# 111 means 1 row, 1 column and cell 1 (It's basically a grid creation)
ax = fig.add_subplot(111)

ax.hist(x, 100)
ax.set_title('Normal distribution with $\mu=0, \sigma=1$')
fig.savefig('matplotlib_histogram.png')
