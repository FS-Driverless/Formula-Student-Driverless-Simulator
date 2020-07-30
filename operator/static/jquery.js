$(document).ready(function() {

    const logs = [];
    let simulatorActive = false;
    let missionActive = false;
    pollServer();

    /**
     * Poll server every 5 seconds
     */
    function pollServer() {
        const accessToken = $('#access-token').val();

        if (accessToken) {
            $.ajax('poll', {
                data: JSON.stringify({access_token: accessToken}),
                contentType: 'application/json',
                type: 'POST',
                success: res => {
                    res.logs.forEach(log => {
                        if (!logs.includes(log)) {
                            logs.push(log);
                            $('.log-window').append(`<p>${log}</p>`);
                        }
                    });

                    if (simulatorActive !== res.simulator_state) {
                        simulatorActive = res.simulator_state;
                        simulatorActive 
                        ? $('#launch-exit').removeClass('btn-primary').addClass('btn-danger').text('EXIT simulator') 
                        : $('#launch-exit').removeClass('btn-danger').addClass('btn-primary').text('LAUNCH simulator');
                    }

                    if (missionActive !== res.interface_state) {
                        missionActive = res.interface_state;
                        missionActive 
                        ? $('#start-stop').removeClass('btn-primary').addClass('btn-danger').text('Send STOP signal') 
                        : $('#start-stop').removeClass('btn-danger').addClass('btn-primary').text('Send GO signal');
                    }
                },
            });
        }
        setTimeout(pollServer, 5000);
    }

    /**
     * Simulator launch exit button handler
     */
    $('#launch-exit').click(() => {
        const accessToken = $('#access-token').val();
        if (accessToken === '') {
            alert('Give an access token.');
            return
        }
    
        const selectedTeam = $("input:radio[name ='team-select']:checked").val();
        if (selectedTeam === undefined) {
            alert('Select a team.');
            return
        }
    
        const selectedMission = $("input:radio[name ='mission-select']:checked").val();
        if (selectedMission === undefined) {
            alert('Select a mission.');
            return
        }
    
        const selectedTrack = $("input:radio[name ='track-select']:checked").val();
        if (selectedTrack === undefined) {
            alert('Select a track.');
            return
        }

        $('#launch-exit').prop('disabled', true);
        $('#launch-exit').blur();
        if(simulatorActive) {
            $.ajax('simulator/exit', {
                data: JSON.stringify({access_token: accessToken}),
                contentType: 'application/json',
                type: 'POST',
                success: res => {
                    simulatorActive = false;
                    logs.push(res.response);
                    $('#launch-exit').prop('disabled', false);
                    $('#launch-exit').removeClass('btn-danger').addClass('btn-primary').text('LAUNCH simulator') 
                    $('.log-window').append(`<p>${res.response}</p>`);
                },
                error: res => {
                    $('#launch-exit').prop('disabled', false);
                    alert(res.responseJSON.error);
                }
            }) 
        } else {
            $.ajax('simulator/launch', {
                data: JSON.stringify({id: selectedTeam, mission: selectedMission, track: selectedTrack, access_token: accessToken}),
                contentType: 'application/json',
                type: 'POST',
                success: res => {
                    simulatorActive = true;
                    logs.push(res.response);
                    $('#launch-exit').prop('disabled', false);
                    $('#launch-exit').removeClass('btn-primary').addClass('btn-danger').text('EXIT simulator'); 
                    $('.log-window').append(`<p>${res.response}</p>`);
                },
                error: res => {
                    $('#start-stop').prop('disabled', false);
                    alert(res.responseJSON.error);
                }
            });
        }
       
    });

    /**
     * ROS bridge start stop button handler
     */
    $('#start-stop').click(() => {
        const accessToken = $('#access-token').val();
        if (accessToken === '') {
            alert('Give an access token.');
            return
        }

        $('#start-stop').prop('disabled', true);
        $('#launch-exit').blur();
        if(missionActive) {
            $.ajax('mission/stop', {
                data: JSON.stringify({access_token: accessToken,  sender: 'operator'}),
                contentType: 'application/json',
                type: 'POST',
                success: res => {
                    $('#start-stop').prop('disabled', false);
                    missionActive = false;
                    logs.push(res.response);
                    $('#start-stop').removeClass('btn-danger').addClass('btn-primary').text('Send GO signal');
                    $('.log-window').append(`<p>${res.response}</p>`);
                },
                error: res => {
                    $('#start-stop').prop('disabled', false);
                    alert(res.responseJSON.error);
                }
            })
        } else {
            $.ajax('mission/start', {
                data: JSON.stringify({access_token: accessToken}),
                contentType: 'application/json',
                type: 'POST',
                success: res => {
                    $('#start-stop').prop('disabled', false);
                    missionActive = true;
                    logs.push(res.response);
                    $('#start-stop').removeClass('btn-primary').addClass('btn-danger').text('Send STOP signal'); 
                    $('.log-window').append(`<p>${res.response}</p>`);
                },
                error: res => {
                    $('#start-stop').prop('disabled', false);
                    alert(res.responseJSON.error);
                }
            });
        }
    });
});