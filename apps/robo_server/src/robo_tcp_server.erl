-module(robo_tcp_server).

-export([start_link/2]).


start_link(ListenSocket, ServerPid) ->
    Pid = spawn_link(fun() ->
        accept_connections(ListenSocket, ServerPid)
    end),
    {ok, Pid}.


accept_connections(ListenSocket, ServerPid) ->
    case gen_tcp:accept(ListenSocket) of
        {ok, Socket} ->
            ConnectionPid =
                spawn(fun() ->
                    handle_connection(Socket, ServerPid)
                end),

            %% Transfer ownership of the socket to the connection process.
            ok = gen_tcp:controlling_process(Socket, ConnectionPid),

            %% Tell the connection process that it now owns the socket.
            ConnectionPid ! socket_ready,

            accept_connections(ListenSocket, ServerPid);

        {error, closed} ->
            ok;

        {error, Reason} ->
            io:format("Accept error: ~p~n", [Reason]),
            accept_connections(ListenSocket, ServerPid)
    end.


handle_connection(Socket, ServerPid) ->
    %% Wait until accept_connections/2 has transferred
    %% ownership of the socket to us.
    receive
        socket_ready ->
            ok
    end,

    %% Now active mode is safe.
    inet:setopts(Socket, [{active, true}]),

    receive
        {tcp, Socket, Data} ->
            Request = binary_to_term(Data),
            handle_initial_request(
                Socket,
                ServerPid,
                Request
            );

        {tcp_closed, Socket} ->
            gen_tcp:close(Socket);

        {tcp_error, Socket, Reason} ->
            io:format("TCP error: ~p~n", [Reason]),
            gen_tcp:close(Socket)
    end.


handle_initial_request(
    Socket,
    ServerPid,
    {hello, RobotId, Current}
) ->
    %% Tell the main server that a robot connected.
    ServerPid ! {
        robot_connected,
        self(),
        RobotId,
        Current
    },

    robot_loop(Socket, ServerPid, RobotId);


handle_initial_request(
    Socket,
    ServerPid,
    {task, Goal}
) ->
    ServerPid ! {
        task_received,
        self(),
        Goal
    },

    task_loop(Socket);


handle_initial_request(
    Socket,
    _ServerPid,
    Request
) ->
    gen_tcp:send(
        Socket,
        term_to_binary(
            {error, {unknown_request, Request}}
        )
    ),
    gen_tcp:close(Socket).


robot_loop(Socket, ServerPid, RobotId) ->
    receive

        %% Main server wants to send something to the robot.
        {send, Response} ->
            case gen_tcp:send(
                Socket,
                term_to_binary(Response)
            ) of
                ok ->
                    robot_loop(
                        Socket,
                        ServerPid,
                        RobotId
                    );

                {error, Reason} ->
                    io:format(
                        "Failed to send to robot ~p: ~p~n",
                        [RobotId, Reason]
                    ),

                    ServerPid ! {
                        robot_disconnected,
                        RobotId
                    },

                    gen_tcp:close(Socket)
            end;


        %% Robot sent something to the server.
        {tcp, Socket, Data} ->
            Request = binary_to_term(Data),

            ServerPid ! {
                robot_message,
                RobotId,
                Request
            },

            robot_loop(
                Socket,
                ServerPid,
                RobotId
            );


        {tcp_closed, Socket} ->
            ServerPid ! {
                robot_disconnected,
                RobotId
            },

            gen_tcp:close(Socket);


        {tcp_error, Socket, Reason} ->
            io:format(
                "TCP error for robot ~p: ~p~n",
                [RobotId, Reason]
            ),

            ServerPid ! {
                robot_disconnected,
                RobotId
            },

            gen_tcp:close(Socket);


        stop ->
            gen_tcp:close(Socket)

    end.


task_loop(Socket) ->
    receive

        {send, Response} ->
            gen_tcp:send(
                Socket,
                term_to_binary(Response)
            ),
            gen_tcp:close(Socket);

        {tcp_closed, Socket} ->
            gen_tcp:close(Socket);

        {tcp_error, Socket, _Reason} ->
            gen_tcp:close(Socket)

    end.