<template>
  <form class="d-flex flex-column border rounded p-3 align-items-start gap-3" style="width: 22rem">
    <h3>Policy</h3>
    <div class="w-100">
      <div class="d-flex justify-content-between gap-3">
        <div class="text-start flex-grow-1">
          <label class="form-label" for="time-start">Start</label>
          <input class="form-control" type="time" id="time-start" @input="this.$emit('start-time-changed', $event.target.value)"/>
        </div>
        <div class="text-start flex-grow-1">
          <label class="form-label" for="time-end">End</label>
          <input class="form-control" type="time" id="time-end" @input="this.$emit('end-time-changed', $event.target.value)"/>
        </div>
      </div>
      <div class="form-text text-start">The dataset may be used between Start and End</div>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="expiration">Expires on</label>
      <input class="form-control" type="date" id="expiration" @input="this.$emit('expires-on-changed', $event.target.value)"/>
      <div class="form-text text-start">Until this time the dataset is available for the user</div>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="max-usages">Number of usages</label>
      <input class="form-control" type="number" id="max-usages" @input="this.$emit('number-of-usages-changed', $event.target.value)"/>
      <div class="form-text text-start">Number of times the dataset may be used</div>
    </div>
      <div class="form-check">
          <input class="form-check-input" type="checkbox" id="local-logging" @input="this.$emit('local-logging-changed', $event.target.checked)">
          <label class="form-check-label" for="local-logging">
            Local logging
          </label>
      </div>
      <div class="form-check">
          <input class="form-check-input" type="checkbox" id="remote-logging" @input="this.$emit('remote-logging-Changed', $event.target.checked)">
          <label class="form-check-label" for="remote-logging">
            Remote logging
          </label>
      </div>
      <div class="form-text text-start">If a value is not set, the corresponding field wont be regarded in the policy</div>
  </form>
</template>

<script>
import axios from 'axios'

export default {
  props: ['startTime', 'endTime', 'expiresOn', 'numberOfUsages', 'localLogging', 'remoteLogging'],
  methods: {
    submit () {
      axios.post('policy', {
        id: '1ead349e-36ec-4d44-abd3-d94c576a74ca',
        event: 'data-access',
        businessHours: {
          start: this.startTime,
          end: this.endTime
        },
        maxUsages: this.numberOfUsages,
        expiresOn: this.expiresOn,
        localLogging: this.localLogging,
        remoteLogging: this.remoteLogging
      }).then(response => {
        console.log('success')
      })
    }
  }
}
</script>
