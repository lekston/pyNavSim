def set_1to1_scale(ax):
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()

    x_center = 0.5*(x_min + x_max)
    y_center = 0.5*(y_min + y_max)

    x_range = x_max - x_min
    y_range = y_max - y_min

    range_new = max(x_range, y_range)

    x_min_new = x_center - 0.5*range_new
    x_max_new = x_center + 0.5*range_new

    y_min_new = y_center - 0.5*range_new
    y_max_new = y_center + 0.5*range_new

    ax.set_xlim(x_min_new, x_max_new)
    ax.set_ylim(y_min_new, y_max_new)