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
      <input class="form-control" type="number" id="max-usages" v-model="usages" @input="this.$emit('max-usages-changed', $event.target.value)"/>
      <div v-if="invalidMaxUsages" class="form-text text-start text-danger">The number of usages must be greater than 0</div>
      <div v-else class="form-text text-start">Number of times the dataset may be used</div>
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
export default {
  props: ['startTime', 'endTime', 'expiresOn', 'maxUsages', 'localLogging', 'remoteLogging'],
  data () {
    return {
      usages: this.maxUsages,
      invalidMaxUsages: false
    }
  },
  watch: {
    usages (value) {
      if (value === '') {
        this.invalidMaxUsages = false
      } else if (value <= 0) {
        this.invalidMaxUsages = true
      } else {
        this.invalidMaxUsages = false
      }
    }
  }
}
</script>
