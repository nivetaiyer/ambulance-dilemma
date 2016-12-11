include("framework_test.jl")




function write_file(pi, f_out)

    open(f_out, "w") do f

        for p in pi

            write(f, "$p\n")

        end

    end

end



# Saving utility values for a number of runs of the simulation

c=1.0

s = (2,5,10,1,0,0) #THIS IS THE STATE WE START FROM

final=[]

push!(final,"iter, depth, exp const, x_car, y_car, x_amb, y_amb, vx_amb, vy_amb, discounted reward")



save_hist = true;

history = [];



for iter = 50:50:300 #20:320

	for d = 1:2:15 #5:60

		(u,h) = the_test(d, iter, c, s)

		push!(final,(iter,d,c,s,u))

        if save_hist

            push!(history, h)

        end
		println("done with $d depth")

	end
    println("done with $iter iterations")
end



write_file(final,"log_batch_1.csv")



# for saving the history from a single simulation

if save_hist

    h_array = []

    push!(h_array, "x_car, y_car,x_amb, y_amb, vx_amb, vy_amb")

    for state in history

        s = state[1]

        #println(typeof(state))

        push!(h_array, [s.x_car, s.y_car, s.x_amb, s.y_amb, s.vx_amb, s.vy_amb])

    end

    write_file(h_array, "history_log_batch_1.csv")

end