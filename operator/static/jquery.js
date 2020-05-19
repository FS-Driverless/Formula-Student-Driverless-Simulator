$(document).ready(function() {

    // Start button handler
    $('#start').click(function() {
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

        $.ajax('mission/start', {
            data: JSON.stringify({id: selectedTeam, mission: selectedMission}),
            contentType: 'application/json',
            type: 'POST',
            success: function(res) {
                $('.log-window').append(`<p>${res.response}</p>`)
            }
        });
    });

    // Stop button handler
    $('#stop').click(function() {
        $.ajax('mission/stop', {
            type: 'POST',
            success: function(res) {
                $('.log-window').append(`<p>${res.response}</p>`)
            }
        });     
    });

    // Reset button handler
    $('#reset').click(function() {
        $.ajax('mission/reset', {
            type: 'POST',
            success: function(res) {
                $('.log-window').append(`<p>${res.response}</p>`)
            }
        });     
    });
});