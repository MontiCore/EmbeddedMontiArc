<template>
  <form class="d-flex flex-column border rounded p-3 align-items-start gap-3" style="width: 22rem">
    <div class="w-100">
      <div class="d-flex justify-content-between gap-3">
        <div class="text-start flex-grow-1">
          <label class="form-label" for="time-start">Start</label>
          <input class="form-control" type="time" id="time-start" v-model="startTime"/>
        </div>
        <div class="text-start flex-grow-1">
          <label class="form-label" for="time-end">End</label>
          <input class="form-control" type="time" id="time-end" v-model="endTime"/>
        </div>
      </div>
      <div class="form-text text-start">The dataset may be used between Start and End</div>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="expiration">Expires on</label>
      <input class="form-control" type="date" id="expiration"/>
      <div class="form-text text-start">Until this time the dataset is available for the user</div>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="max-usages">Number of usages</label>
      <input class="form-control" type="number" id="max-usages" v-model="expiresOn"/>
      <div class="form-text text-start">Number of times the dataset may be used</div>
    </div>
      <div class="form-check">
          <input class="form-check-input" type="checkbox" id="local-logging" v-model="localLogging">
          <label class="form-check-label" for="local-logging">
            Local logging
          </label>
      </div>
      <div class="form-check">
          <input class="form-check-input" type="checkbox" id="remote-logging" v-model="remoteLogging">
          <label class="form-check-label" for="remote-logging">
            Remote logging
          </label>
      </div>
    <button type="submit" class="btn btn-primary fw-bold" @click="submit" style="width: 8rem">Create policy</button>
  </form>
</template>

<script>
import axios from 'axios'

export default {
  data () {
    return {
      startTime: '',
      endTime: '',
      expiresOn: '',
      numberOfUsages: 0,
      localLogging: false,
      remoteLogging: false
    }
  },
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
