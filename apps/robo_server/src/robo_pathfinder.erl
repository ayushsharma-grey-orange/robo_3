-module(robo_pathfinder).

-export([find_path/3]).


find_path(Start, Goal, Obstacles) ->
    H = heuristic(Start, Goal),

    OpenSet = [
        {H, 0, Start, [Start]}
    ],

    ClosedSet = sets:new(),

    search(OpenSet, ClosedSet, Goal, Obstacles).


search([], _ClosedSet, _Goal, _Obstacles) ->
    {error, no_path};

search(OpenSet, ClosedSet, Goal, Obstacles) ->
    {F, G, Current, Path} = pop_lowest_f(OpenSet),

    case Current of
        Goal ->
            {ok, lists:reverse(Path)};

        _ ->
            case sets:is_element(Current, ClosedSet) of
                true ->
                    search(
                        remove_node(OpenSet, F, G, Current, Path),
                        ClosedSet,
                        Goal,
                        Obstacles
                    );

                false ->
                    NewClosedSet =
                        sets:add_element(Current, ClosedSet),

                    RemainingOpenSet =
                        remove_node(
                            OpenSet,
                            F,
                            G,
                            Current,
                            Path
                        ),

                    Neighbours =
                        get_neighbours(Current),

                    ValidNeighbours =
                        [
                            Position ||
                            Position <- Neighbours,
                            valid_position(
                                Position,
                                Obstacles
                            ),
                            not sets:is_element(
                                Position,
                                NewClosedSet
                            )
                        ],

                    NewOpenSet =
                        add_neighbours(
                            ValidNeighbours,
                            G + 1,
                            Goal,
                            Path,
                            RemainingOpenSet
                        ),

                    search(
                        NewOpenSet,
                        NewClosedSet,
                        Goal,
                        Obstacles
                    )
            end
    end.


add_neighbours(
    [],
    _G,
    _Goal,
    _Path,
    OpenSet
) ->
    OpenSet;

add_neighbours(
    [Position | Rest],
    G,
    Goal,
    Path,
    OpenSet
) ->
    H = heuristic(Position, Goal),
    F = G + H,

    NewNode = {
        F,
        G,
        Position,
        [Position | Path]
    },

    NewOpenSet = [NewNode | OpenSet],

    add_neighbours(
        Rest,
        G,
        Goal,
        Path,
        NewOpenSet
    ).


pop_lowest_f([First | Rest]) ->
    pop_lowest_f(Rest, First).


pop_lowest_f([], Lowest) ->
    Lowest;

pop_lowest_f(
    [Current | Rest],
    Lowest
) ->
    {F1, _, _, _} = Current,
    {F2, _, _, _} = Lowest,

    case F1 < F2 of
        true ->
            pop_lowest_f(Rest, Current);

        false ->
            pop_lowest_f(Rest, Lowest)
    end.


remove_node(
    OpenSet,
    F,
    G,
    Position,
    Path
) ->
    lists:delete(
        {F, G, Position, Path},
        OpenSet
    ).


get_neighbours({Row, Col}) ->
    [
        {Row - 1, Col},
        {Row + 1, Col},
        {Row, Col - 1},
        {Row, Col + 1}
    ].


valid_position(
    {Row, Col},
    Obstacles
) ->
    Row >= 1 andalso
    Row =< 9 andalso
    Col >= 1 andalso
    Col =< 9 andalso
    not lists:member(
        {Row, Col},
        Obstacles
    ).


heuristic(
    {Row1, Col1},
    {Row2, Col2}
) ->
    abs(Row1 - Row2) +
    abs(Col1 - Col2).