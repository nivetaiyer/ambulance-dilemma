

using POMDPs
using Distributions
#POMDPs.add("MCTS")

# define state type
# TODO: make everything int?
#v_car = 1
type AmbulanceProblemState
   x_car::Int64 # car position
   y_car::Int64
   #vx_car::Int64 # car velocity
   #vy_car::Int64
   x_amb::Int64  # ambulance position
   y_amb::Int64
   vx_amb::Int64 # ambulance velocity
   vy_amb::Int64
#   done::Bool # are we in a terminal state
end

# functions may want to define later?
# initial state constructor
# TODO: change this. include/disinclude relevant vars
AmbulanceProblemState(x_car::Int64, y_car::Int64, y_amb::Int64) = AmbulanceProblemState(x_car,y_car,0,y_amb,0,0)
# checks if the position of two states are the same
posequal_allstate(s1::AmbulanceProblemState, s2::AmbulanceProblemState) = s1.x_car == s2.x_car && s1.y_car == s2.y_car && s1.x_amb == s2.x_amb && s1.y_amb == s2.y_amb && s1.vx_amb == s2.vx_amb && s1.vy_amb == s2.vy_amb
# copies state s2 to s1
# note: should we use deepcopy()?
function Base.copy!(s1::AmbulanceProblemState, s2::AmbulanceProblemState)
   s1.x_car = s2.x_car
   s1.y_car = s2.y_car
   #s1.vx_car = s2.vx_car
   #s1.vy_car = s2.vy_car
   s1.x_amb = s2.x_amb
   s1.y_amb = s2.y_amb
   s1.vx_amb = s2.vx_amb
   s1.vy_amb = s2.vy_amb
   #s1.done = s2.done
   s1
end
 
# function to determine if we've crashed
#hasCrashed(s::AmbulanceProblemState) = s.x_car == s.x_amb && s.y_car == s.y_amb

# functions for MCTS apparently?
Base.hash(s::AmbulanceProblemState, h::UInt64 = zero(UInt64)) = hash(s.x_car, hash(s.y_car, hash(s.y_amb, hash(s.vy_amb, h))))
function Base.isequal(s1::AmbulanceProblemState, s2::AmbulanceProblemState)
 car_vars = s1.x_car == s2.x_car && s1.y_car == s2.y_car  
 amb_vars = s1.y_amb == s2.y_amb && s1.x_amb == s2.x_amb && s1.vy_amb == s2.vy_amb  && s1.vx_amb == s2.vx_amb

 return car_vars && amb_vars #&& s1.done == s2.done
end  

# define actions
#abstract Action <: Enum
# immutable forward <: Action end # forward throttle
# immutable zero <: Action end # zero throttle
# immutable reverse <: Action end # reverse throttle do we need this?
# immutable rightf <: Action end # Right forward
# immutable leftf <: Action end # Left forward

# the ambulance problem mdp type
type AmbulanceWorld <: MDP{AmbulanceProblemState, Symbol} # Note that our MDP is parametarized by the state and the action
   size_x::Int64 # x size of the grid
   size_y::Int64 # y size of the grid
   size_vx::Int64 # size of velocity space in x
   size_vy::Int64 # size of velocity space in y
   reward_states::Vector{AmbulanceProblemState} # the state in which agent receives reward (neg or pos)
   reward_values::Vector{Float64} # reward values corresponding to those states
   discount_factor::Float64 # discount factor
   amb_length::Int64
end

#we use key worded arguments so we can change any values we pass in
function AmbulanceWorld()
 sx::Int64=15 #size_x
 sy::Int64=15 #size_y
 vx::Int64=7 #size_vx (indexing for velocity starts at 0)
 vy::Int64=3 #size_vy
 rs::Vector{AmbulanceProblemState}=[]
 rv::Vector{Float64}=rv=[] #reward values at goal state
 discount_factor::Float64=0.9
 amb_length::Int64=1

# find reward states
#  vxrange = Int64(-floor(vx/2)):Int64(floor(vx/2))
for x_car = 1:sx, y_car=1:sy, x_amb=1:sx, y_amb=1:sy, vx_amb in [-3,-2, 2, 3], vy_amb in [-3,-2, 2, 3]
     R=0
   # speeding penalty
   if sqrt(vx_amb^2 + vy_amb^2)>=3
       R-=25
   end
   # going fast reward
   R += sqrt(vx_amb^2 + vy_amb^2)*10
   # hospital states
   if (x_amb==9 || x_amb==10 || x_amb==11 || x_amb==12 || x_amb==13 || x_amb==14 || x_amb==15) && (y_amb==15)
     R += 500
   end
   # if we have crashed
   if x_car == x_amb && y_car in [y_amb-1, y_amb, y_amb+1]
       R -= 1000 
   end
   if R>0
      push!(rs, AmbulanceProblemState(x_car, y_car, x_amb, y_amb, vx_amb, vy_amb) )
     push!(rv , R)
   end
  end
  return AmbulanceWorld(sx, sy, vx+4, vy, rs, rv, discount_factor, amb_length) #changed vx
end





# Spaces
type StateSpace <: AbstractSpace
   states::Vector{AmbulanceProblemState}
end

# Do we want to include or loop over the done variable? It creates states where we've crashed but aren't done...

# create an array of ALL OF YOUR STATES
function POMDPs.states(mdp::AmbulanceWorld)
   s = AmbulanceProblemState[]
    for x_car = 1:mdp.size_x, x_amb = 1:mdp.size_x, y_car = 1:mdp.size_y, y_amb = 1:mdp.size_y, vx_amb=1:mdp.size_vx, vy_amb=0:mdp.size_vy
       # x_car::Float64 # car position
       #y_car::Float64
       # vx_car::Float64 # car velocity
       #vy_car::Float64
       #x_amb::Float64  # ambulance position
       #y_amb::Float64
       #vx_amb::Float64 # ambulance velocity
       #vy_amb::Float64
        push!(s, AmbulanceProblemState(x_car, y_car, x_amb, y_amb, vx_amb, vy_amb))
     end
     return StateSpace(s)
end;


# iterator idk
function POMDPs.iterator(space::StateSpace)
    return space.states
end

# sampling function
# NOTE@Niveta: I think this could cause problems if we have states in our state
# space that are unreachable (e.g. where we've crashed but done is false)
# NOTE 2: We pass in a state and then copy the value of something in space.states
# into this "box." because Julia uses pass by reference we don't want to use the
# actual sp that we get from space.states
# NOTE 3: I don't understand exactly how to 'end' works in rand(rng, 1:end)
# NOTE 4: Could dealw ith this by adding an if statement and resampling if we get
# a state that is actaully rteachable
function POMDPs.rand(rng::AbstractRNG, space::StateSpace, s::AmbulanceProblemState)
    sp = space.states[rand(rng, 1:end)]
    copy!(s, sp)
    s
end

## Action Space
type ActionSpace <: AbstractSpace
    actions::Vector{Symbol}
end

function POMDPs.actions(mdp::AmbulanceWorld)
    acts = [:forward, :zero, :reverse, :rightf, :leftf]
    return ActionSpace(acts)
end;
# this is a second defition of the actions function interface that specifies
# the behavior if you pass an mdp, state, and action space
POMDPs.actions(mdp::AmbulanceWorld, s::AmbulanceProblemState, as::ActionSpace=actions(mdp)) = as;

# iterator idk
function POMDPs.iterator(space::ActionSpace)
    return space.actions
end;

# a function to sample from the action space
# NOTE but really a question. Why would you define a sampling function that
# samples actions that you have to PASS an action to? IDK.
# It's implemented in the example so maybe its necessary for the POMDPs interface?
function POMDPs.rand(rng::AbstractRNG, space::ActionSpace, a::Symbol)
    return space.actions[rand(rng, 1:end)]
end;
# a function that samples actions and makes just as little sense
# also we don't have this function AmbulanceWorldAction
function POMDPs.rand(rng::AbstractRNG, space::ActionSpace)
    a = AmbulanceWorldAction(:forward)
    return rand(rng, space, a)
end;

# initializer functions
POMDPs.create_state(mdp::AmbulanceWorld) = AmbulanceProblemState(1,1,1)
POMDPs.create_action(mdp::AmbulanceWorld) = :forward

# Distributions
# What about the 'jumping' problem in the dynamics? How do we deal with propogating velocity?
# Is the way we've done it correct?

#  size_x::Int64 # x size of the grid
#    size_y::Int64 # y size of the grid
#    size_vx::Int64 # size of velocity space in x
#    size_vy::Int64 # size of velocity space in y
#    reward_states::Vector{AmbulanceProblemState} # the state in which agent receives reward (neg or pos)
#    reward_values::Vector{Float64} # reward values corresponding to those states
#    discount_factor::Float64 # discount factor
#    amb_length::Int64

# transition model
# function transition_model()
#   mdp = AmbulanceWorld()
#   # s', ACTION, s
#   # car_x, car_y, amb_x, amb_y, amb_vy, amb_vx, ACTION, car_x, car_y, amb_x, amb_y, amb_vy, amb_vx
#   T = spzeros(mdp.size_x, mdp.size_y, mdp.size_x, mdp.size_y, mdp.size_vy+1, mdp.size_vx, 5, mdp.size_x, mdp.size_y, mdp.size_x, mdp.size_y, mdp.size_vy+1, mdp.size_vx)
#   acts = POMDPs.actions(AmbulanceWorld)
#   for a in acts
#     for vx in 1:mdp.size_vx
#       for vy in 1:mdp.size_vy+1
#         for amb_x in 1:mdp.size_x
#           for amb_y in 1:mdp.size_y
#             for car_x in 1:mdp.size_x
#               for car_y in 1:mdp.size_y
#                 vx_val = vx-4
#                 vy_val = vy-1
#                 if a == forward
#                   # compute next position
#                   # consider limiting range of vy for ambulance so it can't jump sideways
#                   cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
#                   amb_yp = min(amb_y + vy_val, mdp.size_y);
#                   amb_vxp = min(vx_val + 1, 3);
#                   amb_vyp = min(vy_val + 1, 3);
#                 elseif a == zero
#                   # compute next position
#                   # consider limiting range of vy for ambulance so it can't jump sideways
#                   cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
#                   amb_yp = min(amb_y + vy_val, mdp.size_y);
#                   amb_vxp = min(vx_val , 3);
#                   amb_vyp = min(vy_val , 3);
#                 elseif a == reverse
#                   # compute next position
#                   # consider limiting range of vy for ambulance so it can't jump sideways
#                   cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
#                   amb_yp = min(amb_y + vy_val, mdp.size_y);
#                   amb_vxp = min(vx_val - 1, 3);
#                   amb_vyp = min(vy_val - 1, 3);
#                 elseif a == rightf
#                   # compute next position
#                   # consider limiting range of vy for ambulance so it can't jump sideways
#                   cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
#                   amb_yp = min(amb_y + vy_val, mdp.size_y);
#                   amb_vxp = min(vx_val + 1, 3);
#                   amb_vyp = min(vy_val, 3);
#                 elseif a == leftf
#                   # compute next position
#                   # consider limiting range of vy for ambulance so it can't jump sideways
#                   cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
#                   amb_yp = min(amb_y + vy_val, mdp.size_y);
#                   amb_vxp = min(vx_val - 1, 3);
#                   amb_vyp = min(vy_val, 3);
#                 end
#                 T[cxp, car_y, amb_xp, amb_yp, amb_vyp, amb_vxp, a, car_x, car_y, amb_x, amb_y, vy_val, vx_val]=1.0
#               end
#             end
#           end
#         end
#       end
#     end
#   end
#   return T
# end


type AmbulanceWorldDistribution <: AbstractDistribution
  neighbors::Array{AmbulanceProblemState}
  probs::Array{Float64}
  cat::Categorical
end

function POMDPs.create_transition_distribution(mdp::AmbulanceWorld)
  neighbors = [AmbulanceProblemState(i,i,i,i,i,i) for i=1]
  probabilities = ones(1)
  cat = Categorical(1)
  return AmbulanceWorldDistribution(neighbors, probabilities, cat)
end

function POMDPs.iterator(d::AmbulanceWorldDistribution)
  return d.neighbors
end;

function POMDPs.pdf(d::AmbulanceWorldDistribution, s::AmbulanceProblemState)
  for (i,sp) in enumerate d.neighbors
    if s == sp
      return d.probs[i]
    end
  end
  return 0.0
end;

function POMDPs.rand(rng::AbstractRNG, d::AmbulanceWorldDistribution, s::AmbulanceProblemState)
    d.cat = Categorical(d.probs) # init the categorical distribution
    ns = d.neighbors[rand(d.cat)] # sample a neighbor state according to the distribution c
    copy!(s, ns)
    return s # return the pointer to s
end;


function POMDPs.transition(mdp::AmbulanceWorld,
                            state::AmbulanceProblemState,
  action::Symbol,
  d::AmbulanceWorldDistribution = create_transition_distribution(mdp))

  neighbors = d.neighbors
  probability = d.probs

  car_x = state.x_car; amb_x = state.x_amb; amb_y = state.y_amb;
  vx = state.vx_amb; vy = state.vy_amb; car_y = state.y_car;

  a = action

  vx_val = vx
  vy_val = vy
  # if we've crashed, the state doesn't change
  if car_x == amb_x && car_y in [amb_y-1, amb_y, amb_y+1]
    cxp = car_x; amb_xp=amb_x; amb_yp = amb_y;
    amb_vxp = vx_val; amb_vyp = vy_val;
  # if we've reached the hospital, the state doesn't change
  elseif (amb_x in [9,10,11,12,13,14,15]) && (amb_y == 15)
    cxp = car_x; amb_xp=amb_x; amb_yp = amb_y;
    amb_vxp = vx_val; amb_vyp = vy_val;
  elseif a == :forward
    # compute next position
    # consider limiting range of vy for ambulance so it can't jump sideways
    cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
    amb_yp = min(amb_y + vy_val, mdp.size_y);
    amb_vxp = min(vx_val + 1, 3);
    amb_vyp = min(vy_val + 1, 3);
  elseif a == :zero
    # compute next position
    # consider limiting range of vy for ambulance so it can't jump sideways
    cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
    amb_yp = min(amb_y + vy_val, mdp.size_y);
    amb_vxp = min(vx_val , 3);
    amb_vyp = min(vy_val , 3);
  elseif a == :reverse
    # compute next position
    # consider limiting range of vy for ambulance so it can't jump sideways
    cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
    amb_yp = min(amb_y + vy_val, mdp.size_y);
    amb_vxp = min(vx_val - 1, 3);
    amb_vyp = min(vy_val - 1, 3);
  elseif a == :rightf
    # compute next position
    # consider limiting range of vy for ambulance so it can't jump sideways
    cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
    amb_yp = min(amb_y + vy_val, mdp.size_y);
    amb_vxp = min(vx_val + 1, 3);
    amb_vyp = min(vy_val, 3);
  elseif a == :leftf
    # compute next position
    # consider limiting range of vy for ambulance so it can't jump sideways
    cxp = min(car_x + 1, mdp.size_x) ; amb_xp = min(amb_x + vx_val, mdp.size_x);
    amb_yp = min(amb_y + vy_val, mdp.size_y);
    amb_vxp = min(vx_val - 1, 3);
    amb_vyp = min(vy_val, 3);
  end

  probability[1] = 1.0
  neighbors[1].x_car = cxp;
  neighbors[1].y_car = state.y_car
  neighbors[1].x_amb = amb_xp
  neighbors[1].y_amb = amb_yp
  neighbors[1].vx_amb = amb_vxp
  neighbors[1].vy_amb = amb_vyp

  return d

end

function POMDPs.reward(mdp::AmbulanceWorld, state::AmbulanceProblemState, action::Symbol, statep::AmbulanceProblemState )
 # if we've crashed, the state doesn't change
  if state.x_car == state.x_amb && state.y_car in [state.y_amb-1, state.y_amb, state.y_amb+1]
     return 0.0
  # if we've reached the hospital, the state doesn't change
  elseif (state.x_amb in [9,10,11,12,13,14,15]) && (state.y_amb == 15)
     return 0.0
  end
  r = 0.0
  reward_states = mdp.reward_states
  reward_values = mdp.reward_values
  n= length(reward_states)
  for i = 1:n
    if posequal_allstate(state, reward_states[i])
      r += reward_values[i]
    end
  end
  return r
end

POMDPs.discount(mdp::AmbulanceWorld) = mdp.discount_factor;

function the_test(the_depth, num_iterations)

using MCTS

# initialize the problem
mdp = AmbulanceWorld()

# initialize the solver
# the hyper parameters in MCTS can be tricky to set properly
# n_iterations: the number of iterations that each search runs for
# depth: the depth of the tree (how far away from the current state the algorithm explores)
# exploration constant: this is how much weight to put into exploratory actions.
# A good rule of thumb is to set the exploration constant to what you expect the upper bound on your average expected reward to be.
using POMDPToolbox

solver = MCTSSolver(n_iterations=num_iterations, depth=the_depth, exploration_constant=1.0)

# initialize the policy by passing in your problem and the solver
policy = MCTSPolicy(solver, mdp)

hist = HistoryRecorder()

s = AmbulanceProblemState(1,7,15,1,0,0)

r = simulate(hist, mdp, policy, s)

println("Total discounted reward $r")

println(hist.state_hist)

end
