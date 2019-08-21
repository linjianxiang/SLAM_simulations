function robot_plot_update(X_r,shape,plt)

    shape_moved= polar2cart(X_r,shape);
    set(plt,'xdata',shape_moved(1,:),'ydata',shape_moved(2,:));
end