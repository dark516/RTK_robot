from rover_agent_mcp.agent_text_node import AgentTextNode


def test_search_reply_reports_actual_qr_and_return_to_start():
    reply = AgentTextNode._search_result_reply({
        'tool_results': [{
            'name': 'search_static_obstacle',
            'arguments': {
                'static_obstacle': 'желтый дом',
                'target_obstacle': 'building_debris',
            },
            'result': {
                'success': True,
                'inspection': {'target_obstacle_cell': {'class': 'building_debris'}},
                'qr_codes': ['victim-17'],
                'waited_at_qr_sec': 5.0,
                'return_to_start': {'success': True},
            },
        }],
    })

    assert 'желтый дом' in reply
    assert 'обломки здания' in reply
    assert 'victim-17' in reply
    assert '5 секунд' in reply
    assert 'Вернулся в точку «старт»' in reply
