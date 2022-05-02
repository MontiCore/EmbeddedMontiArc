<template>
  <nav class="p-0">
      <ul class="pagination">
        <li class="page-item hover-cursor" :class="{ disabled: currentPage ===  1 }">
          <a class="page-link" aria-label="Previous" @click="leftArrowClicked">
            <span aria-hidden="true">&laquo;</span>
          </a>
        </li>
        <li class="page-item hover-cursor" :class="{ active: currentPage === visiblePages[0], disabled: visiblePages[0] > Math.ceil(elements / 15) }"><a class="page-link" @click="pageLinkClicked(visiblePages[0])">{{ visiblePages[0] }}</a></li>
        <li class="page-item hover-cursor" :class="{ active: currentPage === visiblePages[1], disabled: visiblePages[1] > Math.ceil(elements / 15) }"><a class="page-link" @click="pageLinkClicked(visiblePages[1])">{{ visiblePages[1] }}</a></li>
        <li class="page-item hover-cursor" :class="{ active: currentPage === visiblePages[2], disabled: visiblePages[2] > Math.ceil(elements / 15) }"><a class="page-link" @click="pageLinkClicked(visiblePages[2])">{{ visiblePages[2] }}</a></li>
        <li class="page-item hover-cursor" :class="{ disabled: currentPage ===  Math.ceil(elements / 15) }">
          <a class="page-link" aria-label="Next" @click="rightArrowClicked">
            <span aria-hidden="true">&raquo;</span>
          </a>
        </li>
      </ul>
    </nav>
</template>

<script>
export default {
  props: {
    elements: {
      type: Number,
      required: true
    }
  },
  data () {
    return {
      currentPage: 1,
      visiblePages: [1, 2, 3]
    }
  },
  methods: {
    pageLinkClicked (newPage) {
      this.currentPage = newPage

      if (newPage !== 1) {
        this.visiblePages = [newPage - 1, newPage, newPage + 1]
      }
      this.$emit('pageChange', this.currentPage)
    },
    leftArrowClicked () {
      this.currentPage = 1
      this.visiblePages = [1, 2, 3]
      this.$emit('pageChange', this.currentPage)
    },
    rightArrowClicked () {
      this.currentPage = Math.ceil(this.elements / 15)
      this.visiblePages = [this.currentPage - 1, this.currentPage, this.currentPage + 1]
      this.$emit('pageChange', this.currentPage)
    }
  }
}
</script>

<style scoped>
  .hover-cursor {
    cursor: pointer;
  }
</style>
