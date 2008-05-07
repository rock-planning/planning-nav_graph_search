require 'Tioga/FigureMaker'
require 'stringio'
include Tioga

class Plotter
    include Tioga::FigureConstants

    attr_reader :t
    attr_reader :data
    def initialize(io)
        @t = Tioga::FigureMaker.default

        @data = io.read
        t.def_figure("Graph") { dstar_graph }
        t.def_figure("Costs") { dstar_costs }
    end

    def read_header(io)
        values = io.readline.split(" ")
        x_size, y_size = Float(values[0]), Float(values[1])
        values = io.readline.split(" ")
        x_start, y_start, x_goal, y_goal = Float(values[0]), Float(values[1]), Float(values[2]), Float(values[3])
        return x_size, y_size, x_start, y_start, x_goal, y_goal
    end

    def dstar_graph
        t.line_width = 0

        file = StringIO.new data
        x_size, y_size, x_start, y_start, x_goal, y_goal = read_header file

        t.show_marker 'at' => [x_start / x_size, y_start / y_size],
                'marker' => Bullet, 'scale' => 0.5
        t.show_marker 'at' => [x_goal / x_size, y_goal / y_size],
                'marker' => Bullet, 'scale' => 0.5

        file.each_line do |line|
            if line == "\n"
                break
            end

            values = line.split " "
            x0, y0 = Float(values[0]), Float(values[1])
            x1, y1 = Float(values[3]), Float(values[4])
            t.stroke_line x0/x_size, y0/y_size, x1/x_size, y1/y_size
        end
    end

    def dstar_costs
        max = nil
        t.subplot('right_margin' => 0.07) { max = cost_image }
        t.subplot('left_margin' => 0.95,
                  'top_margin' => 0.05,
                  'bottom_margin' => 0.05) { gradient_bar("Cost", 0, max, t.rainbow_colormap) }
    end

    def cost_image
        t.line_width = 0

        file = StringIO.new data
        x_size, y_size, x_start, y_start, x_goal, y_goal = read_header file

        costs = Dtable.new(x_size, y_size)
        contour_mode = false
        contour = []
        file.each_line do |line|
            if line == "\n"
                contour_mode = true
            end

            values = line.split " "
            x, y = Integer(values[0]), Integer(values[1])
            if contour_mode
                contour << [x, y]
            else
                costs[y_size - 1 - y, x] = Float(values[2])
            end
        end
        max = costs.max
        image = t.create_image_data(costs, 'min_value' => 0, 'max_value' => max)

        t.fill_color = Wheat
        t.fill_frame
        t.show_image(
                'll' => [0, 0],
                'lr' => [1, 0],
                'ul' => [0, 1],
                'color_space' => t.rainbow_colormap,
                'data' => image,
                'w' => x_size, 'h' => y_size)

        contour.each do |x, y|
            t.show_marker 'at' => [Float(x) / x_size, Float(y) / y_size],
                    'marker' => Bullet, 'scale' => 0.1
        end
        t.show_marker 'at' => [Float(x_start) / x_size, Float(y_start) / y_size],
                'marker' => Bullet, 'scale' => 0.5, 'color' => Green
        t.show_marker 'at' => [x_goal / x_size, y_goal / y_size],
                'marker' => Bullet, 'scale' => 0.5, 'color' => Green
        max
    end

    def gradient_bar(title, min, max, colormap)
        xmin = 0; xmax = 1; xmid = 0.5
        t.rescale(0.8)
        t.xaxis_type = AXIS_LINE_ONLY
        t.xaxis_loc  = BOTTOM
        t.top_edge_type = AXIS_LINE_ONLY
        t.yaxis_loc = t.ylabel_side = RIGHT
        t.yaxis_type = AXIS_WITH_TICKS_AND_NUMERIC_LABELS
        t.left_edge_type = AXIS_WITH_TICKS_ONLY
        t.ylabel_shift += 0.5
        t.yaxis_major_tick_length *= 0.6
        t.yaxis_minor_tick_length *= 0.5
        t.do_box_labels(nil, nil, title)
        t.show_plot('boundaries' => [xmin, xmax, min, max]) do
            t.axial_shading(
                'start_point' => [xmid, min],
                'end_point'   => [xmid, max],
                'colormap'    => colormap
            )
        end
    end
end

plotter = Plotter.new STDIN

