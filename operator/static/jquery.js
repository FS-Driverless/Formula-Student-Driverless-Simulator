$(document).ready(function() {

    let simulatorActive = false;
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
                    $('.log-window').html(res.logs.map(log => `<p>${log}</p>`).join(""))

                    if (simulatorActive !== res.simulator_state) {
                        simulatorActive = res.simulator_state;
                        simulatorActive 
                        ? $('#launch-exit').removeClass('btn-primary').addClass('btn-danger').text('EXIT simulator') 
                        : $('#launch-exit').removeClass('btn-danger').addClass('btn-primary').text('LAUNCH simulator');
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
                    $('#launch-exit').prop('disabled', false);
                    $('#launch-exit').removeClass('btn-primary').addClass('btn-danger').text('EXIT simulator'); 
                },
                error: res => {
                    $('#launch-exit').prop('disabled', false);
                    alert(res.responseJSON.error);
                }
            });
        }
       
    });

});