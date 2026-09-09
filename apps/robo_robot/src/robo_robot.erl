-module(robo_robot).

-export([start/2]).

-define(SERVER, "localhost").


% -define(PORT, 5555).
port() -> application:get_env(robo_robot, server_port, 5555).


%% robo_robot.erl
step_delay_ms() -> application:get_env(robo_robot, step_delay_ms, 500).


start(RobotId, Current) ->

    io:format(
      "Robot ~p starting at ~p~n",
      [RobotId, Current]),

    {ok, Socket} =
        gen_tcp:connect(
          ?SERVER,
          port(),
          [binary,
           {packet, 4},
           {active, false}]),

    io:format(
      "Robot ~p connected to server~n",
      [RobotId]),

    %% Register this robot with the server.
    gen_tcp:send(
      Socket,
      term_to_binary(
        {hello, RobotId, Current})),

    case gen_tcp:recv(Socket, 0) of

        {ok, Data} ->

            Response =
                binary_to_term(Data),

            io:format(
              "Robot ~p received: ~p~n",
              [RobotId, Response]),

            wait_for_command(
              Socket,
              RobotId);

        Error ->

            io:format(
              "Robot ~p disconnected while waiting "
              "for initial response: ~p~n",
              [RobotId, Error]),

            gen_tcp:close(Socket)
    end.


%%--------------------------------------------------------------------
%% Wait for commands from server
%%--------------------------------------------------------------------


wait_for_command(Socket,
                 RobotId) ->

    case gen_tcp:recv(Socket, 0) of

        {ok, Data} ->

            Command =
                binary_to_term(Data),

            io:format(
              "Robot ~p received command: ~p~n",
              [RobotId, Command]),

            handle_command(
              Socket,
              RobotId,
              Command),

            wait_for_command(
              Socket,
              RobotId);

        {error, closed} ->

            io:format(
              "Robot ~p disconnected~n",
              [RobotId]),

            gen_tcp:close(Socket);

        {error, Reason} ->

            io:format(
              "Robot ~p TCP error: ~p~n",
              [RobotId, Reason]),

            gen_tcp:close(Socket)
    end.


%%--------------------------------------------------------------------
%% Commands from server
%%--------------------------------------------------------------------


handle_command(Socket,
               RobotId,
               {go, Goal, Positions}) ->

    io:format(
      "Robot ~p assigned goal ~p~n",
      [RobotId, Goal]),

    move_window(
      Socket,
      RobotId,
      Goal,
      Positions);

handle_command(Socket,
               RobotId,
               {wait, Reason}) ->

    io:format(
      "Robot ~p waiting: ~p~n",
      [RobotId, Reason]),

    gen_tcp:send(
      Socket,
      term_to_binary(
        {wait_ack, RobotId}));

handle_command(Socket,
               RobotId,
               Command) ->

    io:format(
      "Robot ~p received unknown command: ~p~n",
      [RobotId, Command]),

    gen_tcp:send(
      Socket,
      term_to_binary(
        {error, {unknown_command, Command}})).


%%--------------------------------------------------------------------
%% Execute one window
%%--------------------------------------------------------------------


move_window(_Socket,
            _RobotId,
            _Goal,
            []) ->
    ok;

%% Last position in the current window.
%%
%% The robot reports that it moved.
%% The server sends back move_ack and then,
%% when the window is complete, sends the next
%% {go, Goal, Window} command.
move_window(Socket,
            RobotId,
            Goal,
            [Position]) ->

    io:format(
      "Robot ~p moving to ~p~n",
      [RobotId, Position]),

    timer:sleep(step_delay_ms()),
    gen_tcp:send(
      Socket,
      term_to_binary(
        {moved, RobotId, Position})),

    case gen_tcp:recv(Socket, 0) of

        {ok, Data} ->

            Response =
                binary_to_term(Data),

            io:format(
              "Robot ~p received: ~p~n",
              [RobotId, Response]),

            case Response of

                {move_ack, Position} ->
                    ok;

                {goal_reached, Position} ->

                    io:format(
                      "Robot ~p reached its goal at ~p~n",
                      [RobotId, Position]),

                    ok;

                {move_denied, Reason} ->

                    io:format(
                      "Robot ~p move denied: ~p~n",
                      [RobotId, Reason]),

                    ok;

                Other ->

                    io:format(
                      "Robot ~p received unexpected response: ~p~n",
                      [RobotId, Other]),

                    ok
            end;

        {error, closed} ->

            io:format(
              "Robot ~p disconnected~n",
              [RobotId]),

            ok;

        {error, Reason} ->

            io:format(
              "Robot ~p TCP error: ~p~n",
              [RobotId, Reason]),

            ok
    end;

%% More positions remain in the current window.
move_window(Socket,
            RobotId,
            Goal,
            [Position | Rest]) ->

    io:format(
      "Robot ~p moving to ~p~n",
      [RobotId, Position]),

    timer:sleep(step_delay_ms()),

    gen_tcp:send(
      Socket,
      term_to_binary(
        {moved, RobotId, Position})),

    case gen_tcp:recv(Socket, 0) of

        {ok, Data} ->

            Response =
                binary_to_term(Data),

            io:format(
              "Robot ~p received: ~p~n",
              [RobotId, Response]),

            case Response of

                {move_ack, Position} ->

                    move_window(
                      Socket,
                      RobotId,
                      Goal,
                      Rest);

                {goal_reached, Position} ->

                    io:format(
                      "Robot ~p reached its goal at ~p~n",
                      [RobotId, Position]),

                    ok;

                {move_denied, Reason} ->

                    io:format(
                      "Robot ~p move denied: ~p~n",
                      [RobotId, Reason]),

                    ok;

                Other ->

                    io:format(
                      "Robot ~p received unexpected response: ~p~n",
                      [RobotId, Other]),

                    ok
            end;

        {error, closed} ->

            io:format(
              "Robot ~p disconnected~n",
              [RobotId]),

            ok;

        {error, Reason} ->

            io:format(
              "Robot ~p TCP error: ~p~n",
              [RobotId, Reason]),

            ok
    end.
