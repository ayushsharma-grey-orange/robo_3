-module(robo_server).

-behaviour(gen_server).

-export([start_link/0, submit_task/1]).
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
         terminate/2, code_change/3]).

% -define(WINDOW_SIZE, 3).

%% Static obstacles for the demo grid.
% -define(OBSTACLES, [{9,3}, {8,3}, {7,3}, {7,2}]).



window_size() -> application:get_env(robo_server, window_size, 3).
obstacles()   -> application:get_env(robo_server, obstacles, []).
port() -> application:get_env(robo_server, tcp_port, 5555).




start_link() ->
    gen_server:start_link({local, ?MODULE}, ?MODULE, [], []).


submit_task(Goal) ->
    gen_server:call(?MODULE, {submit_task, Goal}).


init([]) ->
    {ok, ListenSocket} = gen_tcp:listen(
        port(),
        [binary,
         {packet, 4},
         {active, false},
         {reuseaddr, true}]
    ),

    {ok, _AcceptPid} =
        robo_tcp_server:start_link(ListenSocket, self()),

    Reservations =
        ets:new(robo_reservations, [named_table, set, public]),

    State = #{
        robots => #{},
        tasks => queue:new(),
        obstacles => obstacles(),
        reservations => Reservations
    },

    io:format("Server listening on port 5555~n"),

    {ok, State}.


%%--------------------------------------------------------------------
%% gen_server callbacks
%%--------------------------------------------------------------------

handle_call({submit_task, Goal}, _From, State) ->
    TaskId = erlang:unique_integer([monotonic, positive]),

    Task = #{
        id => TaskId,
        goal => Goal
    },

    Tasks = maps:get(tasks, State),

    NewState =
        process_tasks(
            State#{tasks => queue:in(Task, Tasks)}
        ),

    {reply, {ok, TaskId}, NewState};


handle_call(_Request, _From, State) ->
    {reply, {error, unknown_request}, State}.


handle_cast(_Request, State) ->
    {noreply, State}.


%%--------------------------------------------------------------------
%% Robot connection
%%--------------------------------------------------------------------

handle_info(
    {robot_connected, Handler, RobotId, Current},
    State
) ->
    io:format(
        "Robot ~p connected at ~p~n",
        [RobotId, Current]
    ),

    Handler ! {send, {hello_ack, RobotId}},

    Robots = maps:get(robots, State),

    Robot = #{
        handler => Handler,
        current => Current,
        status => idle,
        goal => undefined,
        path => []
    },

    NewRobots =
        maps:put(RobotId, Robot, Robots),

    NewState =
        State#{robots => NewRobots},

    {noreply, process_tasks(NewState)};


%%--------------------------------------------------------------------
%% Task received over TCP
%%--------------------------------------------------------------------

handle_info(
    {task_received, Handler, Goal},
    State
) ->
    TaskId =
        erlang:unique_integer([monotonic, positive]),

    Handler ! {send, {task_ack, TaskId}},

    Task = #{
        id => TaskId,
        goal => Goal
    },

    Tasks = maps:get(tasks, State),

    NewState =
        process_tasks(
            State#{tasks => queue:in(Task, Tasks)}
        ),

    {noreply, NewState};


%%--------------------------------------------------------------------
%% Robot reports movement
%%--------------------------------------------------------------------

handle_info(
    {robot_message, RobotId, {moved, RobotId, NewPosition}},
    State
) ->
    handle_robot_move(
        RobotId,
        NewPosition,
        State
    );


%%--------------------------------------------------------------------
%% Robot disconnected
%%--------------------------------------------------------------------

handle_info(
    {robot_disconnected, RobotId},
    State
) ->
    io:format(
        "Robot ~p disconnected~n",
        [RobotId]
    ),

    case maps:find(
        RobotId,
        maps:get(robots, State)
    ) of
        {ok, Robot} ->

            NewState0 =
                release_robot_reservations(
                    RobotId,
                    State
                ),

            Robots0 =
                maps:get(robots, NewState0),

            Robots1 =
                maps:remove(
                    RobotId,
                    Robots0
                ),

            NewState1 =
                NewState0#{
                    robots => Robots1
                },

            case maps:get(status, Robot) of

                busy ->
                    Goal = maps:get(goal, Robot),

                    TaskId =
                        erlang:unique_integer(
                            [monotonic, positive]
                        ),

                    Task = #{
                        id => TaskId,
                        goal => Goal
                    },

                    Tasks =
                        maps:get(tasks, NewState1),

                    {noreply,
                     process_tasks(
                         NewState1#{
                             tasks =>
                                 queue:in(
                                     Task,
                                     Tasks
                                 )
                         }
                     )};

                waiting ->
                    Goal = maps:get(goal, Robot),

                    TaskId =
                        erlang:unique_integer(
                            [monotonic, positive]
                        ),

                    Task = #{
                        id => TaskId,
                        goal => Goal
                    },

                    Tasks =
                        maps:get(tasks, NewState1),

                    {noreply,
                     process_tasks(
                         NewState1#{
                             tasks =>
                                 queue:in(
                                     Task,
                                     Tasks
                                 )
                         }
                     )};

                idle ->
                    {noreply, NewState1}
            end;

        error ->
            {noreply, State}
    end;


handle_info(_Info, State) ->
    {noreply, State}.


%%--------------------------------------------------------------------
%% Robot movement handling
%%--------------------------------------------------------------------

handle_robot_move(RobotId, NewPosition, State) ->

    Robots = maps:get(robots, State),

    io:format(
        "Robot ~p requested to move to ~p~n",
        [RobotId, NewPosition]
    ),

    case maps:find(RobotId, Robots) of

        error ->
            {noreply, State};

        {ok, Robot} ->

            Current = maps:get(current, Robot),
            Path = maps:get(path, Robot),
            Goal = maps:get(goal, Robot),

            case expected_next(Current, Path) of

                {ok, NewPosition} ->

                    Table =
                        maps:get(
                            reservations,
                            State
                        ),

                    case ets:lookup(
                        Table,
                        NewPosition
                    ) of

                        %%------------------------------------------------
                        %% Correct movement and correct reservation
                        %%------------------------------------------------
                        [{NewPosition, RobotId}] ->

                            %% The robot has now entered this position,
                            %% so its reservation can be released.
                            release_position(
                                NewPosition,
                                RobotId,
                                State
                            ),

                            UpdatedRobot =
                                Robot#{
                                    current =>
                                        NewPosition
                                },

                            NewRobots =
                                maps:put(
                                    RobotId,
                                    UpdatedRobot,
                                    Robots
                                ),

                            case NewPosition =:= Goal of

                                %%----------------------------------------
                                %% Goal reached
                                %%----------------------------------------
                                true ->

                                    Handler =
                                        maps:get(
                                            handler,
                                            Robot
                                        ),

                                    Handler !
                                        {send,
                                         {goal_reached,
                                          NewPosition}},

                                    io:format(
                                        "Robot ~p reached goal ~p~n",
                                        [RobotId, Goal]
                                    ),

                                    FinalRobot =
                                        UpdatedRobot#{
                                            status => idle,
                                            goal => undefined,
                                            path => []
                                        },

                                    FinalRobots =
                                        maps:put(
                                            RobotId,
                                            FinalRobot,
                                            NewRobots
                                        ),

                                    NewState =
                                        State#{
                                            robots =>
                                                FinalRobots
                                        },

                                    {noreply,
                                     process_tasks(
                                         NewState
                                     )};


                                %%----------------------------------------
                                %% Normal movement
                                %%----------------------------------------
                                false ->

                                    Handler =
                                        maps:get(
                                            handler,
                                            Robot
                                        ),

                                    %% Acknowledge the movement first.
                                    Handler !
                                        {send,
                                         {move_ack,
                                          NewPosition}},

                                    NewState =
                                        State#{
                                            robots =>
                                                NewRobots
                                        },

                                    %% If this was the last reserved
                                    %% position of the current window,
                                    %% the server automatically sends
                                    %% the next window.
                                    case has_reservations(
                                        RobotId,
                                        NewState
                                    ) of

                                        true ->
                                            {
                                                noreply,
                                                process_waiting_robots(
                                                    NewState
                                                )
                                            };

                                        false ->
                                            io:format(
                                                "Window complete for ~p, "
                                                "sending next window~n",
                                                [RobotId]
                                            ),

                                            NextState =
                                                send_next_window(
                                                    RobotId,
                                                    NewState
                                                ),

                                            {
                                                noreply,
                                                process_waiting_robots(
                                                    NextState
                                                )
                                            }
                                    end
                            end;


                        %%------------------------------------------------
                        %% Position belongs to another robot
                        %%------------------------------------------------
                        [{NewPosition, OtherRobot}] ->

                            Handler =
                                maps:get(
                                    handler,
                                    Robot
                                ),

                            Handler !
                                {send,
                                 {move_denied,
                                  {position_reserved_by,
                                   OtherRobot}}},

                            {
                                noreply,
                                replan_robot(
                                    RobotId,
                                    State
                                )
                            };


                        %%------------------------------------------------
                        %% Position was not reserved
                        %%------------------------------------------------
                        [] ->

                            Handler =
                                maps:get(
                                    handler,
                                    Robot
                                ),

                            Handler !
                                {send,
                                 {move_denied,
                                  position_not_reserved}},

                            {
                                noreply,
                                replan_robot(
                                    RobotId,
                                    State
                                )
                            }
                    end;


                %%--------------------------------------------------------
                %% Robot tried to move somewhere other than its path
                %%--------------------------------------------------------
                {ok, ExpectedPosition} ->

                    Handler =
                        maps:get(
                            handler,
                            Robot
                        ),

                    Handler !
                        {send,
                         {move_denied,
                          {unexpected_position,
                           ExpectedPosition}}},

                    {
                        noreply,
                        replan_robot(
                            RobotId,
                            State
                        )
                    };


                {error, Reason} ->

                    Handler =
                        maps:get(
                            handler,
                            Robot
                        ),

                    Handler !
                        {send,
                         {move_denied,
                          Reason}},

                    {noreply, State}
            end
    end.


%%--------------------------------------------------------------------
%% Task assignment
%%--------------------------------------------------------------------

process_tasks(State) ->

    Tasks = maps:get(tasks, State),
    Robots = maps:get(robots, State),

    case find_idle_robot(Robots) of

        none ->
            State;

        {ok, RobotId} ->

            case queue:out(Tasks) of

                {empty, _} ->
                    State;

                {{value, Task}, NewTasks} ->

                    Goal =
                        maps:get(
                            goal,
                            Task
                        ),

                    Robot =
                        maps:get(
                            RobotId,
                            Robots
                        ),

                    Current =
                        maps:get(
                            current,
                            Robot
                        ),

                    case Current =:= Goal of

                        true ->

                            io:format(
                                "Task ~p already completed by "
                                "robot ~p at ~p~n",
                                [
                                    maps:get(id, Task),
                                    RobotId,
                                    Goal
                                ]
                            ),

                            process_tasks(
                                State#{
                                    tasks => NewTasks
                                }
                            );

                        false ->

                            case assign_task(
                                RobotId,
                                Goal,
                                State
                            ) of

                                {ok, NewState} ->

                                    process_tasks(
                                        NewState#{
                                            tasks =>
                                                NewTasks
                                        }
                                    );

                                {error, no_path, NewState} ->

                                    io:format(
                                        "No path for task ~p -> ~p~n",
                                        [
                                            maps:get(id, Task),
                                            Goal
                                        ]
                                    ),

                                    %% Keep task queued.
                                    NewState#{
                                        tasks =>
                                            queue:in_r(
                                                Task,
                                                NewTasks
                                            )
                                    }
                            end
                    end
            end
    end.


find_idle_robot(Robots) ->

    case lists:dropwhile(
        fun({_Id, Robot}) ->
            maps:get(status, Robot) =/= idle
        end,
        maps:to_list(Robots)
    ) of

        [{RobotId, _} | _] ->
            {ok, RobotId};

        [] ->
            none
    end.


%%--------------------------------------------------------------------
%% Assign a new task to a robot
%%--------------------------------------------------------------------

assign_task(RobotId, Goal, State) ->

    Robots = maps:get(robots, State),
    Robot = maps:get(RobotId, Robots),

    Current = maps:get(current, Robot),
    Obstacles = maps:get(obstacles, State),

    case robo_pathfinder:find_path(
        Current,
        Goal,
        Obstacles
    ) of

        {ok, Path} ->

            UpdatedRobot =
                Robot#{
                    status => busy,
                    goal => Goal,
                    path => Path
                },

            NewRobots =
                maps:put(
                    RobotId,
                    UpdatedRobot,
                    Robots
                ),

            State1 =
                State#{
                    robots => NewRobots
                },

            case reserve_next_window(
                RobotId,
                State1
            ) of

                {ok, State2, Window} ->

                    Handler =
                        maps:get(
                            handler,
                            Robot
                        ),

                    Handler !
                        {send,
                         {go,
                          Goal,
                          Window}},

                    io:format(
                        "Assigned robot ~p: ~p -> ~p~n",
                        [
                            RobotId,
                            Current,
                            Goal
                        ]
                    ),

                    io:format(
                        "Window for ~p: ~p~n",
                        [
                            RobotId,
                            Window
                        ]
                    ),

                    {ok, State2};

                {error, no_path, State2} ->
                    {error, no_path, State2}
            end;

        {error, no_path} ->
            {error, no_path, State}
    end.


%%--------------------------------------------------------------------
%% Send next window
%%--------------------------------------------------------------------

send_next_window(RobotId, State) ->

    case maps:find(
        RobotId,
        maps:get(robots, State)
    ) of

        error ->
            State;

        {ok, Robot} ->

            case maps:get(status, Robot) of

                busy ->

                    case reserve_next_window(
                        RobotId,
                        State
                    ) of

                        {ok, NewState, Window} ->

                            Handler =
                                maps:get(
                                    handler,
                                    Robot
                                ),

                            Goal =
                                maps:get(
                                    goal,
                                    Robot
                                ),

                            Handler !
                                {send,
                                 {go,
                                  Goal,
                                  Window}},

                            io:format(
                                "Next window for ~p: ~p~n",
                                [
                                    RobotId,
                                    Window
                                ]
                            ),

                            NewState;

                        {error, no_path, NewState} ->

                            io:format(
                                "Robot ~p cannot reserve "
                                "next window; waiting~n",
                                [RobotId]
                            ),

                            WaitingRobot =
                                Robot#{
                                    status => waiting
                                },

                            Robots =
                                maps:get(
                                    robots,
                                    NewState
                                ),

                            NewRobots =
                                maps:put(
                                    RobotId,
                                    WaitingRobot,
                                    Robots
                                ),

                            NewState#{
                                robots => NewRobots
                            }
                    end;

                _ ->
                    State
            end
    end.


%%--------------------------------------------------------------------
%% Reservation handling
%%--------------------------------------------------------------------

% reserve_next_window(RobotId, State) ->

%     Robots = maps:get(robots, State),
%     Robot = maps:get(RobotId, Robots),

%     Current = maps:get(current, Robot),
%     Path = maps:get(path, Robot),

%     case next_positions(
%         Current,
%         Path,
%         window_size()
%     ) of

%         {ok, Positions} ->

%             case reserve_window(
%                 Positions,
%                 RobotId,
%                 State
%             ) of

%                 ok ->
%                     {ok, State, Positions};

%                 {error, _Blocked} ->
%                     replan_and_reserve(
%                         RobotId,
%                         State
%                     )
%             end;

%         {error, goal_reached} ->
%             {error, no_path, State};

%         {error, _Reason} ->
%             replan_and_reserve(
%                 RobotId,
%                 State
%             )
%     end.

reserve_next_window(RobotId, State) ->
    Robots = maps:get(robots, State),
    Robot = maps:get(RobotId, Robots),
    Current = maps:get(current, Robot),
    Path = maps:get(path, Robot),

    case next_positions(Current, Path, ?WINDOW_SIZE) of
        {ok, Positions} ->
            case reserve_window(Positions, RobotId, State) of
                ok ->
                    {ok, store_window(RobotId, Positions, State), Positions};
                {error, _Blocked} ->
                    replan_and_reserve(RobotId, State)
            end;
        {error, goal_reached} ->
            {error, no_path, State};
        {error, _Reason} ->
            replan_and_reserve(RobotId, State)
    end.

store_window(RobotId, Window, State) ->
    Robots = maps:get(robots, State),
    Robot = maps:get(RobotId, Robots),
    UpdatedRobot = Robot#{window => Window},
    State#{robots => maps:put(RobotId, UpdatedRobot, Robots)}.


replan_robot(RobotId, State) ->

    State1 =
        release_robot_reservations(
            RobotId,
            State
        ),

    case maps:find(
        RobotId,
        maps:get(robots, State1)
    ) of

        error ->
            State1;

        {ok, Robot} ->

            case maps:get(status, Robot) of

                busy ->

                    case replan_and_reserve(
                        RobotId,
                        State1
                    ) of

                        {ok, NewState, Window} ->

                            Handler =
                                maps:get(
                                    handler,
                                    Robot
                                ),

                            Goal =
                                maps:get(
                                    goal,
                                    Robot
                                ),

                            Handler !
                                {send,
                                 {go,
                                  Goal,
                                  Window}},

                            NewState;

                        {error, no_path, NewState} ->

                            Robots =
                                maps:get(
                                    robots,
                                    NewState
                                ),

                            UpdatedRobot =
                                Robot#{
                                    status => waiting
                                },

                            NewState#{
                                robots =>
                                    maps:put(
                                        RobotId,
                                        UpdatedRobot,
                                        Robots
                                    )
                            }
                    end;

                _ ->
                    State1
            end
    end.


replan_and_reserve(RobotId, State) ->

    Robots = maps:get(robots, State),
    Robot = maps:get(RobotId, Robots),

    Current = maps:get(current, Robot),
    Goal = maps:get(goal, Robot),

    Obstacles = maps:get(obstacles, State),

    Reserved =
        reserved_positions(
            RobotId,
            State
        ),

    ReplanObstacles =
        lists:usort(
            Obstacles ++ Reserved
        ),

    case robo_pathfinder:find_path(
        Current,
        Goal,
        ReplanObstacles
    ) of

        {ok, Path} ->

            UpdatedRobot =
                Robot#{
                    path => Path
                },

            NewRobots =
                maps:put(
                    RobotId,
                    UpdatedRobot,
                    Robots
                ),

            State1 =
                State#{
                    robots => NewRobots
                },

            case next_positions(
                Current,
                Path,
                window_size()
            ) of

                {ok, Positions} ->

                    case reserve_window(
                        Positions,
                        RobotId,
                        State1
                    ) of

                        ok ->
                            {ok,
                             State1,
                             Positions};

                        {error, _} ->
                            {error,
                             no_path,
                             State1}
                    end;

                {error, _} ->
                    {error,
                     no_path,
                     State1}
            end;

        {error, no_path} ->
            {error, no_path, State}
    end.


reserve_window(
    Positions,
    RobotId,
    State
) ->

    Table =
        maps:get(
            reservations,
            State
        ),

    reserve_window(
        Positions,
        RobotId,
        Table,
        []
    ).


reserve_window(
    [],
    _RobotId,
    _Table,
    _Reserved
) ->
    ok;


reserve_window(
    [Position | Rest],
    RobotId,
    Table,
    Reserved
) ->

    case ets:insert_new(
        Table,
        {Position, RobotId}
    ) of

        true ->

            reserve_window(
                Rest,
                RobotId,
                Table,
                [Position | Reserved]
            );

        false ->

            %% Roll back reservations made for
            %% this window if one position failed.
            lists:foreach(
                fun(P) ->
                    case ets:lookup(
                        Table,
                        P
                    ) of

                        [{P, RobotId}] ->
                            ets:delete(Table, P);

                        _ ->
                            ok
                    end
                end,
                Reserved
            ),

            {error, Position}
    end.


release_position(
    Position,
    RobotId,
    State
) ->

    Table =
        maps:get(
            reservations,
            State
        ),

    case ets:lookup(
        Table,
        Position
    ) of

        [{Position, RobotId}] ->
            ets:delete(
                Table,
                Position
            );

        _ ->
            ok
    end.


release_robot_reservations(
    RobotId,
    State
) ->

    Table =
        maps:get(
            reservations,
            State
        ),

    lists:foreach(
        fun({Position, ReservedBy}) ->

            case ReservedBy =:= RobotId of

                true ->
                    ets:delete(
                        Table,
                        Position
                    );

                false ->
                    ok
            end
        end,
        ets:tab2list(Table)
    ),

    State.


%% Does this robot still have positions reserved?
has_reservations(RobotId, State) ->

    Table =
        maps:get(
            reservations,
            State
        ),

    lists:any(
        fun({_Position, ReservedBy}) ->
            ReservedBy =:= RobotId
        end,
        ets:tab2list(Table)
    ).


%% Return all positions reserved by OTHER robots.
reserved_positions(RobotId, State) ->

    Table =
        maps:get(
            reservations,
            State
        ),

    [
        Position
        ||
        {Position, OtherRobot} <-
            ets:tab2list(Table),
        OtherRobot =/= RobotId
    ].


%%--------------------------------------------------------------------
%% Waiting robots
%%--------------------------------------------------------------------

process_waiting_robots(State) ->

    Robots =
        maps:get(
            robots,
            State
        ),

    lists:foldl(
        fun({RobotId, Robot}, AccState) ->

            case maps:get(
                status,
                Robot
            ) of

                waiting ->
                    replan_waiting_robot(
                        RobotId,
                        AccState
                    );

                _ ->
                    AccState
            end
        end,
        State,
        maps:to_list(Robots)
    ).


replan_waiting_robot(
    RobotId,
    State
) ->

    Robots =
        maps:get(
            robots,
            State
        ),

    Robot =
        maps:get(
            RobotId,
            Robots
        ),

    Current =
        maps:get(
            current,
            Robot
        ),

    Goal =
        maps:get(
            goal,
            Robot
        ),

    Obstacles =
        maps:get(
            obstacles,
            State
        ),

    Reserved =
        reserved_positions(
            RobotId,
            State
        ),

    ReplanObstacles =
        lists:usort(
            Obstacles ++ Reserved
        ),

    case robo_pathfinder:find_path(
        Current,
        Goal,
        ReplanObstacles
    ) of

        {ok, Path} ->

            UpdatedRobot =
                Robot#{
                    status => busy,
                    path => Path
                },

            State1 =
                State#{
                    robots =>
                        maps:put(
                            RobotId,
                            UpdatedRobot,
                            Robots
                        )
                },

            case reserve_next_window(
                RobotId,
                State1
            ) of

                {ok, State2, Window} ->

                    Handler =
                        maps:get(
                            handler,
                            Robot
                        ),

                    Handler !
                        {send,
                         {go,
                          Goal,
                          Window}},

                    io:format(
                        "Waiting robot ~p can move again. "
                        "New window: ~p~n",
                        [
                            RobotId,
                            Window
                        ]
                    ),

                    State2;

                {error, no_path, State2} ->
                    State2
            end;

        {error, no_path} ->
            State
    end.


%%--------------------------------------------------------------------
%% Path helpers
%%--------------------------------------------------------------------

expected_next(Current, Path) ->

    case lists:dropwhile(
        fun(Position) ->
            Position =/= Current
        end,
        Path
    ) of

        [_Current, Next | _] ->
            {ok, Next};

        [_Current] ->
            {error, goal_reached};

        [] ->
            {error, current_position_not_in_path}
    end.


next_positions(
    Current,
    Path,
    WindowSize
) ->

    case lists:dropwhile(
        fun(Position) ->
            Position =/= Current
        end,
        Path
    ) of

        [] ->
            {error, current_position_not_in_path};

        [_Current | Remaining] ->

            case lists:sublist(
                Remaining,
                WindowSize
            ) of

                [] ->
                    {error, goal_reached};

                Positions ->
                    {ok, Positions}
            end
    end.


%%--------------------------------------------------------------------
%% Shutdown / upgrade
%%--------------------------------------------------------------------

terminate(_Reason, State) ->

    case maps:find(
        reservations,
        State
    ) of

        {ok, Table} ->
            ets:delete(Table);

        error ->
            ok
    end,

    ok.


code_change(
    _OldVsn,
    State,
    _Extra
) ->
    {ok, State}.