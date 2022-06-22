<template>
  <div class="d-flex flex-column align-items-center gap-3">
    <div class="card" style="border-color: transparent; border-radius: 0.5rem">
      <table class="table table-striped table-hover rounded">
        <thead class="colored-thead">
        <tr>
          <th class="text-nowrap text-light" v-for="(_, key) in truckData[0]" :key="key">
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
    <TablePagination :elements="elements" @pageChange="pageChanged"/>
  </div>
</template>

<script>
import TablePagination from '@/components/TablePagination.vue'

export default {
  components: {
    TablePagination
  },
  data () {
    return {
      currentPage: 1
    }
  },
  computed: {
    truckData () {
      return this.$store.getters.getDatarows.slice((this.currentPage - 1) * 15, (this.currentPage - 1) * 15 + 15)
    },
    elements () {
      return this.$store.getters.getDatarows.length
    }
  },
  methods: {
    pageChanged (newPage) {
      this.currentPage = newPage
    }
  }
}
</script>

<style scoped>
.colored-thead {
  background-color: #212529;
}

.table-hover tbody tr:hover td, .table-hover tbody tr:hover th {
  background-color: rgb(221, 221, 221);
}
</style>
