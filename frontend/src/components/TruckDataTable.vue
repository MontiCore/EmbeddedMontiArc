<template>
  <div style="overflow-x: scroll">
      <table class="table table-hover">
        <thead>
          <tr>
            <th class="text-nowrap" v-for="(_, key) in truckData[0]" :key="key">
              {{ key }}
            </th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="row in truckData" :key="row">
            <td
              class="text-nowrap"
              v-for="(value, key) in row"
              :key="key"
              style="border-width: 0"
            >
              {{ value }}
            </td>
          </tr>
        </tbody>
      </table>

    </div>
</template>

<script>
export default {
  props: {
    truckData: {
      type: Array,
      required: true
    }
  }
}
</script>
