<template>
  <div>
    <label for="file-upload"  class="d-flex flex-column justify-content-center align-items-center shadow text-start gap-2 p-3 add-card">
        <h1 class="m-0">Add Offer</h1>
        <Icon icon="akar-icons:plus" style="font-size: 3rem" />
    </label>
    <input id="file-upload" ref="fileUpload" type="file" @change="uploadImage()"/>
  </div>
</template>

<script>
import { Icon } from '@iconify/vue'

export default {
  components: {
    Icon
  },
  methods: {
    uploadImage () {
      const file = this.$refs.fileUpload.files[0]
      this.$store.dispatch('uploadCsv', file)
      this.$router.push('/create-offer/rows')
    }
  }
}
</script>

<style scoped>
.add-card {
  border-radius: 0.5rem;
  width: 18rem;
  height: 18rem;
  background-color: rgb(236, 236, 236);
  color: rgb(170, 170, 170);
  cursor: pointer;
}

.add-card:hover {
  color: rgb(140, 140, 140);
}

input[type="file"] {
  display: none;
}
</style>
